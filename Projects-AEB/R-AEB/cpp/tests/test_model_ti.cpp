
#if !defined(VISPER_ARCH_TI)
#error "test_model_ti.cpp must be built with ARCH=ti"
#endif

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include "common/logger.hpp"
#include "infer/infer_ti_impl.hpp"

// -------------------- 单个子进程的测试逻辑 --------------------
// 直接测试通用 TIDLModel，流程：构造 -> SmokeTest -> 析构
static int RunOneProcessSmokeTest(int procIndex,
                                  const std::string& modelPath,
                                  const std::string& tiTarget,
                                  int smokeIters)
{
    try {
        LOG_INFO("[Proc %d] Init TIDLModel, model=%s, ti_target=%s",
                 procIndex, modelPath.c_str(), tiTarget.c_str());

        TIDLModel model(modelPath, tiTarget);

        LOG_INFO("[Proc %d] Start TIDLModel::smokeTest(%d)", procIndex, smokeIters);

        model.smokeTest(smokeIters);

        LOG_INFO("[Proc %d] SmokeTest done", procIndex);
        LOG_INFO("[Proc %d] Exit success", procIndex);
        return 0;
    }
    catch (const std::exception& e) {
        LOG_ERROR("[Proc %d] Exception: %s", procIndex, e.what());
    }
    catch (...) {
        LOG_ERROR("[Proc %d] Unknown exception", procIndex);
    }
    return 1;
}

// -------------------- 命令行解析 --------------------
static void PrintUsage(const char *prog)
{
    std::cout << "Usage: " << prog
              << " -m <model_path> [-p <process_count>] [-n <iters_per_process>] [-d <target>]\n"
              << "  -m <model_path>       Path to TI offline model file or artifact directory\n"
              << "  -p <process_count>    Number of processes to fork (>0), default: 1\n"
              << "  -n <iters_per_proc>   SmokeTest iterations per process (>0), default: 20\n"
              << "  -d, --ti-target       TI TIDL target, default: DSP_C7-2\n";
}

// -------------------- 主函数：多进程测试通用 TIDLModel --------------------
int main(int argc, char** argv)
{
    INIT_LOG("", "", LogLevel::INFO);

    std::string modelPath;
    std::string tiTarget = "DSP_C7-2";
    int processCount = 1;
    int itersPerProc = 20;

    // 简单手写参数解析（不用 getopt / argparse）
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-m" && i + 1 < argc) {
            modelPath = argv[++i];
        } else if (arg == "-p" && i + 1 < argc) {
            processCount = std::stoi(argv[++i]);
        } else if (arg == "-n" && i + 1 < argc) {
            itersPerProc = std::stoi(argv[++i]);
        } else if ((arg == "-d" || arg == "--ti-target" || arg == "--ti_target") && i + 1 < argc) {
            tiTarget = argv[++i];
        } else {
            PrintUsage(argv[0]);
            return 1;
        }
    }

    if (modelPath.empty() || processCount <= 0 || itersPerProc <= 0) {
        PrintUsage(argv[0]);
        return 1;
    }

    LOG_INFO("[Parent] Model: %s", modelPath.c_str());
    LOG_INFO("[Parent] TI target: %s", tiTarget.c_str());
    LOG_INFO("[Parent] Processes: %d, iters per process: %d", processCount, itersPerProc);

    // -p 1 时直接在当前进程执行，避免 fork 对运行时状态的影响
    if (processCount == 1) {
        LOG_INFO("[Parent] processCount=1, run in single process (no fork)");
        const int ret = RunOneProcessSmokeTest(0, modelPath, tiTarget, itersPerProc);
        LOG_INFO("[Parent] Single-process run finished, code=%d", ret);
        return ret;
    }

    // fork 多个子进程
    for (int i = 0; i < processCount; ++i) {
        pid_t pid = fork();
        if (pid < 0) {
            // fork 失败
            const int e = errno;
            LOG_ERROR("[Parent] fork failed: errno=%d (%s) index=%d", e, std::strerror(e), i);
            continue;
        }

        if (pid == 0) {
            // 子进程：跑自己的 Init+SmokeTest，然后退出
            int ret = RunOneProcessSmokeTest(i, modelPath, tiTarget, itersPerProc);
            _exit(ret);  // 用 _exit 避免重复执行父进程的析构等
        }

        LOG_INFO("[Parent] Created child pid=%d (index=%d)", (int)pid, i);
    }

    // 父进程：等待所有子进程结束
    int status = 0;
    pid_t wpid = 0;
    while ((wpid = wait(&status)) > 0) {
        if (WIFEXITED(status)) {
            int code = WEXITSTATUS(status);
            LOG_INFO("[Parent] Child %d exited with code %d", (int)wpid, code);
        }
        else if (WIFSIGNALED(status)) {
            int sig = WTERMSIG(status);
            LOG_WARNING("[Parent] Child %d killed by signal %d", (int)wpid, sig);
        }
        else {
            LOG_WARNING("[Parent] Child %d exited with unknown status=0x%x", (int)wpid, status);
        }
    }

    LOG_INFO("[Parent] All children finished");
    return 0;
}
