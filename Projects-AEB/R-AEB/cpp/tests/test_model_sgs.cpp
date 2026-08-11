
#if !defined(VISPER_ARCH_SGS)
#error "test_model_sgs.cpp must be built with ARCH=sgs"
#endif

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

#include "infer/infer_api.h"
#include "common/logger.hpp"

// -------------------- 单个子进程的测试逻辑 --------------------
// 只通过 infer_* API，流程：Init -> SmokeTest -> 结束
static int RunOneProcessSmokeTest(int procIndex,
                                  const std::string& modelPath,
                                  int smokeIters)
{
    auto safeDeinit = [procIndex]() {
        try {
            infer_DeinitRAEB();
            LOG_INFO("[Proc %d] infer_DeinitRAEB done", procIndex);
        }
        catch (const std::exception& e) {
            LOG_WARNING("[Proc %d] infer_DeinitRAEB exception: %s", procIndex, e.what());
        }
        catch (...) {
            LOG_WARNING("[Proc %d] infer_DeinitRAEB unknown exception", procIndex);
        }
    };

    try {
        // 每次启动前先显式清理一次（幂等），避免重复 init 时遗留资源
        safeDeinit();

        LOG_INFO("[Proc %d] InitRAEB, model=%s", procIndex, modelPath.c_str());

        // 初始化模型（内部会 new IPUModel 等）
        infer_InitRAEB(modelPath);

        LOG_INFO("[Proc %d] Start infer_SmokeTestRAEB(%d)", procIndex, smokeIters);

        // 冒烟测试：内部会随机构造输入并多次调用 IPU 推理
        infer_SmokeTestRAEB(smokeIters);

        // 正常结束也显式释放，避免后续 _exit 跳过静态析构导致资源不回收
        safeDeinit();

        LOG_INFO("[Proc %d] SmokeTest done", procIndex);
        LOG_INFO("[Proc %d] Exit success", procIndex);
        return 0;
    }
    catch (const std::exception& e) {
        safeDeinit();
        LOG_ERROR("[Proc %d] Exception: %s", procIndex, e.what());
    }
    catch (...) {
        safeDeinit();
        LOG_ERROR("[Proc %d] Unknown exception", procIndex);
    }
    return 1;
}

// -------------------- 命令行解析 --------------------
static void PrintUsage(const char *prog)
{
    std::cout << "Usage: " << prog
              << " -m <model_path> [-p <process_count>] [-n <iters_per_process>]\n"
              << "  -m <model_path>       Path to RAEB offline model file\n"
              << "  -p <process_count>    Number of processes to fork (>0), default: 1\n"
              << "  -n <iters_per_proc>   SmokeTest iterations per process (>0), default: 20\n";
}

// -------------------- 主函数：多进程测试 infer_* API --------------------
int main(int argc, char** argv)
{
    std::string modelPath;
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
    LOG_INFO("[Parent] Processes: %d, iters per process: %d", processCount, itersPerProc);

    // -p 1 时直接在当前进程执行，避免 fork 对运行时状态的影响
    if (processCount == 1) {
        LOG_INFO("[Parent] processCount=1, run in single process (no fork)");
        const int ret = RunOneProcessSmokeTest(0, modelPath, itersPerProc);
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
            int ret = RunOneProcessSmokeTest(i, modelPath, itersPerProc);
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
