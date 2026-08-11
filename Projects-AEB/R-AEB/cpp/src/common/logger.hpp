#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <memory>
#include <filesystem>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <thread>
#include <utility>
#include <cstdio>       // for std::snprintf
#include <system_error> // for std::error_code
#ifdef __linux__
#include <dlfcn.h>  // dladdr, Dl_info
#include <cerrno>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

// ===================== 自定义 SourceLoc（C++17 调用点信息） =====================

struct SourceLoc {
    const char* file;
    const char* func;
    int line;
};

// 日志级别
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

// ANSI 颜色
class LogColors {
public:
    static constexpr const char* BLACK = "\033[30m";
    static constexpr const char* RED = "\033[91m";
    static constexpr const char* GREEN = "\033[92m";
    static constexpr const char* YELLOW = "\033[93m";
    static constexpr const char* BLUE = "\033[94m";
    static constexpr const char* MAGENTA = "\033[95m";
    static constexpr const char* CYAN = "\033[96m";
    static constexpr const char* WHITE = "\033[97m";
    static constexpr const char* RESET = "\033[0m";
};

class Logger {
public:
    // 单例
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    // 对应 Python:
    void init_logger(const std::string& name = "",
                     const std::string& save_path = "",
                     LogLevel level = LogLevel::INFO,
                     bool detailed = true) {
        // 1) 写入基础配置
        {
            std::lock_guard<std::mutex> guard(mutex_);
            name_ = name;
            level_ = level;
            detailed_ = detailed;
        }

        // 2) init 摘要
        log_to_console_only(
            LogLevel::INFO,
            "logger: init name=\"" + name + "\" level=" + level_name_short(level) +
            " detailed=" + (detailed ? "on" : "off"),
            SourceLoc{__FILE__, __func__, __LINE__}
        );

        // 3) 文件输出
        if (save_path.empty()) {
            log_to_console_only(
                LogLevel::INFO,
                "logger: file off",
                SourceLoc{__FILE__, __func__, __LINE__}
            );
        } else {
            auto parent = fs::path(save_path).parent_path();
            if (!parent.empty()) fs::create_directories(parent);

            auto ofs = std::make_unique<std::ofstream>(save_path, std::ios::out | std::ios::app);
            bool file_ok = false;
            {
                std::lock_guard<std::mutex> guard(sink_mutex_);
                if (!ofs || !ofs->is_open()) file_stream_.reset();
                else {
                    file_stream_ = std::move(ofs);
                    file_ok = true;
                }
            }

            if (!file_ok) {
                log_to_console_only(
                    LogLevel::ERROR,
                    "logger: file fail path=" + save_path,
                    SourceLoc{__FILE__, __func__, __LINE__}
                );
            } else {
                std::error_code ec;
                std::string abs_path = fs::absolute(fs::path(save_path), ec).string();
                if (ec) abs_path = save_path;

                log_to_console_only(
                    LogLevel::INFO,
                    "logger: file ok path=" + abs_path,
                    SourceLoc{__FILE__, __func__, __LINE__}
                );
            }
        }

        // 4) marker 覆盖 level（前后对比）
        LogLevel before;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            before = level_;
        }

        try {
            set_level_from_file("./");
        } catch (const std::exception& e) {
            log_to_console_only(
                LogLevel::WARNING,
                std::string("logger: set_level_from_file failed: ") + e.what(),
                SourceLoc{__FILE__, __func__, __LINE__}
            );
        } catch (...) {
            log_to_console_only(
                LogLevel::WARNING,
                "logger: set_level_from_file failed: unknown exception",
                SourceLoc{__FILE__, __func__, __LINE__}
            );
        }

        LogLevel after;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            after = level_;
        }

        if (after != before) {
            log_to_console_only(
                LogLevel::INFO,
                "logger: level marker " + level_name_short(before) + "->" + level_name_short(after),
                SourceLoc{__FILE__, __func__, __LINE__}
            );
        } else {
            log_to_console_only(
                LogLevel::INFO,
                "logger: level " + level_name_short(after),
                SourceLoc{__FILE__, __func__, __LINE__}
            );
        }
    }

    // 额外控制：用单独的 setter
    void set_level(LogLevel lvl) {
        std::lock_guard<std::mutex> guard(mutex_);
        level_ = lvl;
    }

    void set_detailed(bool v) {
        std::lock_guard<std::mutex> guard(mutex_);
        detailed_ = v;
    }

    // 控制“控制台是否使用颜色”
    void set_use_color(bool v) {
        std::lock_guard<std::mutex> guard(mutex_);
        console_use_color_ = v;
    }

    // Enable asynchronous sinks so logging does not block latency-sensitive callers.
    // When the queue is full, DEBUG records are dropped instead of blocking.
    void set_async(bool enabled, std::size_t max_queue = 4096) {
        if (enabled) {
            {
                std::lock_guard<std::mutex> guard(async_mutex_);
                async_max_queue_ = max_queue;
                async_stop_.store(false, std::memory_order_release);
                if (!async_worker_.joinable()) {
                    async_worker_ = std::thread(&Logger::async_worker_loop, this);
                }
            }
            async_enabled_.store(true, std::memory_order_release);
            return;
        }

        async_enabled_.store(false, std::memory_order_release);
        stop_async_worker();
    }

    // 通过标记文件控制 level：
    // - criticall / errorl / warningl / infol / debugl
    // - 若存在，覆盖当前 level，并打 INFO 日志
    void set_level_from_file(const std::string& directory = "./") {
        // 收集要检查的目录：工作目录 +（Linux）模块(.so/可执行文件)所在目录
        std::vector<fs::path> dirs;
        dirs.emplace_back(fs::path(directory));

#ifdef __linux__
        {
            std::string mod_dir = module_dir_linux();
            if (!mod_dir.empty()) {
                fs::path mp(mod_dir);

                // 去重（尽量 canonical 后比较；失败就直接字符串比较）
                std::error_code ec1, ec2;
                fs::path a = fs::weakly_canonical(dirs[0], ec1);
                fs::path b = fs::weakly_canonical(mp, ec2);
                bool same = (!ec1 && !ec2) ? (a == b) : (dirs[0] == mp);

                if (!same) dirs.emplace_back(std::move(mp));
            }
        }
#endif

        auto exists_in_any_dir = [&](const char* name, fs::path& hit_dir) -> bool {
            for (const auto& d : dirs) {
                std::error_code ec;
                if (fs::exists(d / name, ec) && !ec) {
                    hit_dir = d;
                    return true;
                }
            }
            return false;
        };

        // 读当前 level（避免无锁读写 race）
        LogLevel cur_level;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            cur_level = level_;
        }

        // 文件优先级：debugl 最高 -> criticall 最低
        struct Marker {
            const char* fname;
            LogLevel lvl;
            const char* msg;
        };

        static constexpr Marker markers[] = {
            {"debugl", LogLevel::DEBUG, "Log level set to [DEBUG]."},
            {"infol", LogLevel::INFO, "Log level set to [INFO]."},
            {"warningl", LogLevel::WARNING, "Log level set to [WARNING]."},
            {"errorl", LogLevel::ERROR, "Log level set to [ERROR]."},
            {"criticall", LogLevel::CRITICAL, "Log level set to [CRITICAL]."},
        };

        bool found = false;
        LogLevel new_level = cur_level;
        std::string msg;
        fs::path hit_dir;

        for (const auto& m : markers) {
            fs::path d;
            if (exists_in_any_dir(m.fname, d)) {
                found = true;
                new_level = m.lvl;
                msg = m.msg;
                hit_dir = std::move(d);
                break; // 关键：按优先级顺序，命中即停
            }
        }

        if (!found) return;

        {
            std::lock_guard<std::mutex> guard(mutex_);
            level_ = new_level;
        }

        // 可选：打印命中目录（方便定位到底是哪边的标记生效）
        std::error_code ec;
        fs::path absd = fs::absolute(hit_dir, ec);
        std::string full_msg = msg + " (marker dir: " + (ec ? hit_dir.string() : absd.string()) + ")";

        // 注意：如果你把 level 设成 ERROR/CRITICAL，这条 INFO 可能被过滤掉；
        // 如果你希望“每次识别到标记必提示”，可以把这里改成 LogLevel::CRITICAL 或单独绕过过滤。
        log_to_console_only(
            LogLevel::INFO,
            full_msg,
            SourceLoc{__FILE__, __func__, __LINE__}
        );
    }

    // ===================== 对外日志接口（printf 风格） =====================
    // 显式带 SourceLoc 的接口（给宏 / 内部用）

    template <typename... Args>
    void debug(const SourceLoc& loc,
               const char* fmt, Args&&... args) {
        logf(LogLevel::DEBUG, loc, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(const SourceLoc& loc,
              const char* fmt, Args&&... args) {
        logf(LogLevel::INFO, loc, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warning(const SourceLoc& loc,
                 const char* fmt, Args&&... args) {
        logf(LogLevel::WARNING, loc, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(const SourceLoc& loc,
               const char* fmt, Args&&... args) {
        logf(LogLevel::ERROR, loc, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(const SourceLoc& loc,
                  const char* fmt, Args&&... args) {
        logf(LogLevel::CRITICAL, loc, fmt, std::forward<Args>(args)...);
    }

    void debug(const SourceLoc& loc,
               const std::string& msg) {
        log_plain(LogLevel::DEBUG, loc, msg);
    }

    void info(const SourceLoc& loc,
              const std::string& msg) {
        log_plain(LogLevel::INFO, loc, msg);
    }

    void warning(const SourceLoc& loc,
                 const std::string& msg) {
        log_plain(LogLevel::WARNING, loc, msg);
    }

    void error(const SourceLoc& loc,
               const std::string& msg) {
        log_plain(LogLevel::ERROR, loc, msg);
    }

    void critical(const SourceLoc& loc,
                  const std::string& msg) {
        log_plain(LogLevel::CRITICAL, loc, msg);
    }

    // ===================== 链式计时器：logger.timer() =====================
    class ChainTimer {
    public:
        explicit ChainTimer(LogLevel lvl = LogLevel::DEBUG)
            : logger_(Logger::instance()),
              level_(lvl),
              start_(std::chrono::steady_clock::now()),
              last_(start_) {}

        ChainTimer& operator()(const std::string& name,
                               const SourceLoc& loc) {
            checkpoint(name, loc);
            return *this;
        }

        void checkpoint(const std::string& name,
                        const SourceLoc& loc) {
            auto now = std::chrono::steady_clock::now();
            auto span = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_).count();
            total_ms_ += span;

            std::ostringstream oss;
            oss << name << ": " << span << " ms, (total: " << total_ms_ << " ms)";
            logger_.log_impl(level_, oss.str(), loc);
            // Exclude logger sink latency from the next measured segment.
            last_ = std::chrono::steady_clock::now();
        }

        void done(const std::string& name,
                  const SourceLoc& loc) {
            checkpoint(name, loc);
        }

    private:
        Logger& logger_;
        LogLevel level_;
        std::chrono::steady_clock::time_point start_;
        std::chrono::steady_clock::time_point last_;
        long long total_ms_{0};
    };

    ChainTimer timer(LogLevel lvl = LogLevel::DEBUG) {
        return ChainTimer(lvl);
    }

private:
    Logger()
        : name_(),
          level_(LogLevel::INFO),
          detailed_(true),
          console_use_color_(true),
          console_out_(&std::cout),
          file_stream_(nullptr) {}

    ~Logger() {
        async_enabled_.store(false, std::memory_order_release);
        stop_async_worker();
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // ============ 工具函数 ============

    template <typename... Args>
    static std::string format_string(const char* fmt, Args&&... args) {
        if constexpr (sizeof...(Args) == 0) {
            return std::string(fmt);
        } else {
            int size = std::snprintf(nullptr, 0, fmt, args...);
            if (size <= 0) {
                return std::string(fmt);
            }
            std::string buf(static_cast<size_t>(size) + 1, '\0');
            std::snprintf(buf.data(), buf.size(), fmt, args...);
            buf.resize(static_cast<size_t>(size));
            return buf;
        }
    }

    static std::string level_name_short(LogLevel lvl) {
        switch (lvl) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::CRITICAL: return "CRIT";
        }
        return "UNK";
    }

    static const char* level_color(LogLevel lvl) {
        switch (lvl) {
        case LogLevel::DEBUG: return LogColors::BLUE;
        case LogLevel::INFO: return LogColors::GREEN;
        case LogLevel::WARNING: return LogColors::YELLOW;
        case LogLevel::ERROR: return LogColors::RED;
        case LogLevel::CRITICAL: return LogColors::MAGENTA;
        }
        return LogColors::WHITE;
    }

    static std::string now_string() {
        using namespace std::chrono;
        auto now = system_clock::now();

        std::time_t t = system_clock::to_time_t(now);
        std::tm tm_buf{};
#if defined(_WIN32)
        if (localtime_s(&tm_buf, &t) != 0) {
            return {};
        }
#else
        if (localtime_r(&t, &tm_buf) == nullptr) {
            return {};
        }
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }

    static std::string file_name(const char* path) {
        std::string p(path);
        auto pos = p.find_last_of("/\\");
        if (pos == std::string::npos) return p;
        return p.substr(pos + 1);
    }


#ifdef __linux__
    // 让 dladdr 有一个明确的、属于当前模块(.so 或 exe)的符号可用
    static void module_anchor() {}

    static std::string module_dir_linux() {
        Dl_info info{};
        // 用锚点函数地址查所属模块路径
        if (dladdr(reinterpret_cast<void*>(&Logger::module_anchor), &info) != 0 && info.dli_fname) {
            std::error_code ec;
            fs::path p(info.dli_fname);

            // 尽量规整成绝对/规范路径（失败也没关系）
            fs::path canon = fs::weakly_canonical(p, ec);
            if (!ec) p = canon;

            return p.parent_path().string();
        }
        return {};
    }
#endif

    template <typename... Args>
    void logf(LogLevel lvl,
              const SourceLoc& loc,
              const char* fmt, Args&&... args) {
        std::string msg = format_string(fmt, std::forward<Args>(args)...);
        log_impl(lvl, msg, loc);
    }

    void log_plain(LogLevel lvl,
                   const SourceLoc& loc,
                   const std::string& msg) {
        log_impl(lvl, msg, loc);
    }

    struct LogRecord {
        LogLevel level;
        bool use_color;
        std::vector<std::string> lines;
    };

    static std::string format_console_line(const LogRecord& record,
                                           const std::string& line) {
        std::string text;
        if (record.use_color) {
            text += level_color(record.level);
        }
        text += line;
        if (record.use_color) {
            text += LogColors::RESET;
        }
        text.push_back('\n');
        return text;
    }

    std::vector<std::string> build_lines(LogLevel lvl,
                                         const std::string& msg,
                                         const SourceLoc& loc,
                                         bool detailed) {
        static constexpr int MAX_LEVEL_BRACKET_WIDTH = 7;

        std::string ts = now_string();
        std::string lvl_name = level_name_short(lvl);

        std::ostringstream oss_level;
        oss_level << "[" << lvl_name << "]";
        std::string level_bracket = oss_level.str();

        std::ostringstream oss_level_padded;
        oss_level_padded << std::right << std::setw(MAX_LEVEL_BRACKET_WIDTH)
            << level_bracket;
        std::string padded_level = oss_level_padded.str();

        std::ostringstream oss_prefix;
        oss_prefix << "[" << ts << "] "
            << padded_level << " ";
        if (detailed) {
            oss_prefix << "[" << file_name(loc.file) << ":"
                << loc.func << ":" << loc.line << "] - ";
        } else {
            oss_prefix << "- ";
        }
        std::string prefix = oss_prefix.str();

        std::vector<std::string> lines;
        std::size_t start = 0;
        while (true) {
            auto pos = msg.find('\n', start);
            if (pos == std::string::npos) {
                lines.emplace_back(prefix + msg.substr(start));
                break;
            }
            lines.emplace_back(prefix + msg.substr(start, pos - start));
            start = pos + 1;
        }
        return lines;
    }

    void log_impl(LogLevel lvl,
                  const std::string& msg,
                  const SourceLoc& loc) {
        bool detailed = true;
        bool use_color = true;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            if (lvl < level_) return;
            detailed = detailed_;
            use_color = console_use_color_;
        }

        LogRecord record{lvl, use_color, build_lines(lvl, msg, loc, detailed)};
        if (async_enabled_.load(std::memory_order_acquire)) {
            enqueue_async(std::move(record));
            return;
        }

        write_record(record);
    }

    void log_to_console_only(LogLevel lvl,
                             const std::string& msg,
                             const SourceLoc& loc) {
        bool detailed = true;
        bool use_color = true;
        {
            std::lock_guard<std::mutex> guard(mutex_);
            if (lvl < level_) return;
            detailed = detailed_;
            use_color = console_use_color_;
        }

        LogRecord record{lvl, use_color, build_lines(lvl, msg, loc, detailed)};
        write_console_record(record);
    }

    void write_console_record(const LogRecord& record) {
        std::lock_guard<std::mutex> guard(sink_mutex_);
        if (!console_out_) return;

        for (const auto& line : record.lines) {
            write_console_text_locked(format_console_line(record, line));
        }
    }

    void write_record(const LogRecord& record) {
        std::lock_guard<std::mutex> guard(sink_mutex_);

        if (console_out_) {
            for (const auto& line : record.lines) {
                write_console_text_locked(format_console_line(record, line));
            }
        }

        if (file_stream_ && file_stream_->is_open()) {
            for (const auto& line : record.lines) {
                (*file_stream_) << line << std::endl;
            }
        }
    }

    void enqueue_async(LogRecord&& record) {
        {
            std::lock_guard<std::mutex> guard(async_mutex_);
            if (async_stop_.load(std::memory_order_acquire) || !async_worker_.joinable()) {
                return;
            }

            if (async_queue_.size() >= async_max_queue_) {
                if (record.level == LogLevel::DEBUG) {
                    return;
                }
                async_queue_.pop_front();
            }

            async_queue_.emplace_back(std::move(record));
        }
        async_cv_.notify_one();
    }

    void async_worker_loop() {
        for (;;) {
            LogRecord record;
            {
                std::unique_lock<std::mutex> lock(async_mutex_);
                async_cv_.wait(lock, [&] {
                    return async_stop_.load(std::memory_order_acquire) || !async_queue_.empty();
                });

                if (async_queue_.empty()) {
                    if (async_stop_.load(std::memory_order_acquire)) break;
                    continue;
                }

                record = std::move(async_queue_.front());
                async_queue_.pop_front();
            }

            write_record(record);
        }
    }

    void stop_async_worker() {
        {
            std::lock_guard<std::mutex> guard(async_mutex_);
            async_stop_.store(true, std::memory_order_release);
        }
        async_cv_.notify_all();

        if (async_worker_.joinable()) {
            async_worker_.join();
        }

        std::lock_guard<std::mutex> guard(async_mutex_);
        async_queue_.clear();
    }

    void write_console_text_locked(const std::string& text) {
#ifdef __linux__
        int fd = -1;
        if (console_out_ == &std::cout) {
            fd = STDOUT_FILENO;
        } else if (console_out_ == &std::cerr) {
            fd = STDERR_FILENO;
        }

        if (fd >= 0) {
            const char* data = text.data();
            std::size_t remaining = text.size();
            while (remaining > 0) {
                const ssize_t written = ::write(fd, data, remaining);
                if (written > 0) {
                    data += written;
                    remaining -= static_cast<std::size_t>(written);
                    continue;
                }
                if ((written < 0) && (errno == EINTR)) {
                    continue;
                }
                break;
            }
            return;
        }
#endif
        (*console_out_) << text;
        console_out_->flush();
    }

private:
    std::string name_;

    LogLevel level_;
    bool detailed_;
    bool console_use_color_;

    std::ostream* console_out_;
    std::unique_ptr<std::ofstream> file_stream_;

    std::mutex mutex_;
    std::mutex sink_mutex_;

    std::atomic<bool> async_enabled_{false};
    std::atomic<bool> async_stop_{false};
    std::mutex async_mutex_;
    std::condition_variable async_cv_;
    std::deque<LogRecord> async_queue_;
    std::thread async_worker_;
    std::size_t async_max_queue_{4096};
};

// 全局别名：像 Python 一样直接用 logger.xxx()
inline Logger& logger = Logger::instance();

// ===================== 宏：对外唯一接口 =====================

// 初始化宏，参数直接透传给 Logger::init_logger
// 示例：INIT_LOG("test", "", LogLevel::DEBUG, true);
#define INIT_LOG(...)          logger.init_logger(__VA_ARGS__)

// 源位置宏（C++17 版）
#define SRC_LOC SourceLoc{__FILE__, __func__, __LINE__}

// 普通日志宏：自动带上调用点
#define LOG_DEBUG(fmt, ...)    logger.debug   (SRC_LOC, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)     logger.info    (SRC_LOC, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)  logger.warning (SRC_LOC, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)    logger.error   (SRC_LOC, fmt, ##__VA_ARGS__)
#define LOG_CRITICAL(fmt, ...) logger.critical(SRC_LOC, fmt, ##__VA_ARGS__)

// 计时器宏：创建 / 打点 / 收尾
// LOG_TIMER_CREATE(t)                    -> auto t = logger.timer();
// LOG_TIMER_CREATE(t, LogLevel::INFO)    -> auto t = logger.timer(LogLevel::INFO);
#define LOG_TIMER_CREATE(var, name, ...)  LOG_DEBUG("%s", name);auto var = logger.timer(__VA_ARGS__)
#define LOG_TIMER(var, name)        (var)(name, SRC_LOC)
#define LOG_TIMER_DONE(var, name)   (var).done(name, SRC_LOC)
