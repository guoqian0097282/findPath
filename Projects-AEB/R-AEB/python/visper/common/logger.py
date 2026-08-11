from __future__ import annotations
r"""全局日志记录器

Constants
---------
- logger: 全局日志记录器

Main API
---------
- logger.init_logger(name, save_path, level): 初始化logger对象的一些参数
- logger.info()
- logger.warning()
- logger.error()
- logger.critical()

"""
import ast
import functools
import logging
import os
import pprint
import sys
import time
from logging.handlers import RotatingFileHandler, MemoryHandler

CRITICAL = logging.CRITICAL
FATAL = logging.FATAL
ERROR = logging.ERROR
WARNING = logging.WARNING
WARN = logging.WARN
INFO = logging.INFO
DEBUG = logging.DEBUG
NOTSET = logging.NOTSET


# ANSI颜色代码
class LogColors:
    BLACK = '\033[30m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    RESET = '\033[0m'


class CustomFormatter(logging.Formatter):
    """自定义日志格式器，支持彩色输出和详细/简洁格式切换"""
    LEVEL_COLORS = {
        logging.DEBUG: LogColors.BLUE,
        logging.INFO: LogColors.GREEN,
        logging.WARNING: LogColors.YELLOW,
        logging.ERROR: LogColors.RED,
        logging.CRITICAL: LogColors.MAGENTA,
    }

    MAX_LEVEL_BRACKET_WIDTH = max(
        len(f'[{lvl}]') for lvl in ["DEBUG", "INFO", "ERROR"]
    )

    def __init__(
            self,
            datefmt: str | None = None,
            use_color: bool = True,
            detailed: bool = True,
    ) -> None:
        super().__init__(datefmt=datefmt)
        self.use_color = use_color
        self.detailed = detailed

    def format(self, record: logging.LogRecord) -> str:
        # 1. 先格式化 message，并尝试对末尾的字典/列表 pprint
        msg = record.getMessage()
        try:
            start = msg.rfind('{')
            if start != -1:
                literal = msg[start:]
                obj = ast.literal_eval(literal)
                pretty = pprint.pformat(obj, width=80)
                msg = msg[:start] + '\n' + pretty
        except Exception:
            pass

        record.msg = msg
        record.args = ()

        # 2. 构造各部分
        # 时间部分
        timestamp = self.formatTime(record, self.datefmt)
        time_part = f'[{timestamp}]'

        # 1）把 修改日志级别名，仅仅是为了保证对齐
        if record.levelname == "WARNING":
            record.levelname = "WARN"
        if record.levelname == "CRITICAL":
            record.levelname = "CRIT"
        # 右对齐后的 “[LEVEL]” 整体
        level_bracket = f'[{record.levelname}]'
        padded_level = level_bracket.rjust(self.MAX_LEVEL_BRACKET_WIDTH)

        if self.detailed:
            # 详细模式，包含文件名、函数名、行号
            file_part = f'[{record.filename}:{record.funcName}:{record.lineno}] - '
        else:
            # 简洁模式，仅保留分隔符
            file_part = '- '

        # 3. 每行加前缀
        lines = msg.split('\n')
        formatted = '\n'.join(
            f"{time_part} {padded_level} {file_part}{line}"
            for line in lines
        )

        # 4. 上色（可选）
        if self.use_color:
            color = self.LEVEL_COLORS.get(record.levelno, LogColors.WHITE)
            formatted = f"{color}{formatted}{LogColors.RESET}"

        return formatted


class Logger:
    """封装全局日志记录器，支持控制台和文件输出"""

    def __init__(self) -> None:
        self.logger = logging.getLogger('default_logger')
        self.logger.setLevel(logging.DEBUG if sys.gettrace() else logging.INFO)

        # 初始化 console handler，默认详细模式
        self.console_handler = logging.StreamHandler()
        self.console_handler.setFormatter(
            CustomFormatter(
                datefmt='%Y-%m-%d %H:%M:%S',
                use_color=True,
                detailed=True,
            )
        )
        if not any(isinstance(h, logging.StreamHandler) for h in self.logger.handlers):
            self.logger.addHandler(self.console_handler)

        self.initialized = False
        self.path_error_reported = False

        # 模块 import 时自动根据目录中的标记文件设置日志级别
        try:
            self._set_level_from_file()  # 默认当前工作目录 "./"
        except Exception:
            # 防止某些极端场景（权限/文件系统问题）导致 import 失败
            pass

    def init_logger(
            self,
            name: str | None = None,
            save_path: str | None = None,
            level: int | None = None,
            detailed: bool = True,
    ) -> None:
        """
        初始化 logger：
        - name: 日志器名称
        - save_path: 日志文件路径
        - level: 日志级别
        - detailed: 是否输出详细格式
        注意：
        - 如存在标记文件（debugl/errorl/...），文件指定的级别会覆盖这里的 level。
        """
        self.initialized = True

        # 名称与级别
        if name:
            self.logger.name = name
        if level:
            self.logger.setLevel(level)

        # 更新 console formatter
        self.console_handler.setFormatter(
            CustomFormatter(
                datefmt='%Y-%m-%d %H:%M:%S',
                use_color=True,
                detailed=detailed,
            )
        )

        # 文件输出
        if save_path:
            log_dir = os.path.dirname(save_path)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)

            # 移除旧的文件/内存 handler
            for h in list(self.logger.handlers):
                if isinstance(h, (RotatingFileHandler, MemoryHandler)):
                    self.logger.removeHandler(h)

            file_handler = RotatingFileHandler(
                save_path,
                maxBytes=10 * 1024 * 1024,
                backupCount=5,
                encoding='utf-8',
            )
            file_handler.setFormatter(
                CustomFormatter(
                    datefmt='%Y-%m-%d %H:%M:%S',
                    use_color=False,
                    detailed=detailed,
                )
            )

            mem_handler = MemoryHandler(
                capacity=1000,
                flushLevel=logging.WARNING,
                target=file_handler,
                flushOnClose=True,
            )
            self.logger.addHandler(mem_handler)
            self.info(f"日志文件保存路径：{os.path.abspath(save_path)}")

        # 再检查一次标记文件，如存在则覆盖上面的 level 设置
        try:
            self._set_level_from_file()
        except Exception:
            pass

    # 新增：从目录中的标记文件设置日志级别 ---------------------------
    def _set_level_from_file(self, directory: str = "./") -> None:
        """
        检查指定目录下是否存在以下文件：
        - criticall -> CRITICAL
        - errorl    -> ERROR
        - warningl  -> WARNING
        - infol     -> INFO
        - debugl    -> DEBUG

        若存在多个，只按优先顺序匹配第一个。
        若都不存在，则保持当前日志级别不变。
        """

        def exists(name: str) -> bool:
            return os.path.exists(os.path.join(directory, name))

        level = None
        msg = ""

        if exists("criticall"):
            level = logging.CRITICAL
            msg = "根据文件 [criticall] 设置日志级别为 [CRITICAL]。"
        elif exists("errorl"):
            level = logging.ERROR
            msg = "根据文件 [errorl] 设置日志级别为 [ERROR]。"
        elif exists("warningl"):
            level = logging.WARNING
            msg = "根据文件 [warningl] 设置日志级别为 [WARNING]。"
        elif exists("infol"):
            level = logging.INFO
            msg = "根据文件 [infol] 设置日志级别为 [INFO]。"
        elif exists("debugl"):
            level = logging.DEBUG
            msg = "根据文件 [debugl] 设置日志级别为 [DEBUG]。"

        if level is not None:
            # 真正设置底层 logger 级别
            self.logger.setLevel(level)
            # 这里直接用底层 logger.log，避免触发 _check_initialized 的“未初始化”提示
            self.logger.log(logging.INFO, msg, stacklevel=2)

    def _check_initialized(self) -> None:
        if self.path_error_reported:
            return

        if not self.initialized:
            self.path_error_reported = True
            self.warning("logger 未初始化，请先调用 init_logger()；日志将不会保存到文件。")
        elif not any(isinstance(h, (RotatingFileHandler, MemoryHandler)) for h in self.logger.handlers):
            self.path_error_reported = True
            self.warning("未指定日志文件保存路径(save_path)；日志将不会保存到文件。")

    def _format_args(self, args):
        """对 args 中的 dict/list/tuple/set 做 pprint.pformat"""
        formatted = []
        for arg in args:
            if isinstance(arg, (dict, list, tuple, set)):
                # 使用 pformat 生成带换行和缩进的字符串
                formatted.append(pprint.pformat(arg, width=80, compact=False))
            else:
                formatted.append(arg)
        return tuple(formatted)

    def debug(self, msg, *args, **kwargs):
        self._check_initialized()
        args = self._format_args(args)
        self.logger.debug(msg, *args, stacklevel=2, **kwargs)

    def info(self, msg, *args, **kwargs):
        self._check_initialized()
        args = self._format_args(args)
        self.logger.info(msg, *args, stacklevel=2, **kwargs)

    def warning(self, msg, *args, **kwargs):
        self._check_initialized()
        args = self._format_args(args)
        self.logger.warning(msg, *args, stacklevel=2, **kwargs)

    def error(self, msg, *args, **kwargs):
        self._check_initialized()
        args = self._format_args(args)
        self.logger.error(msg, *args, stacklevel=2, **kwargs)

    def critical(self, msg, *args, **kwargs):
        self._check_initialized()
        args = self._format_args(args)
        self.logger.critical(msg, *args, stacklevel=2, **kwargs)

    def measure_time(self, func: callable) -> callable:
        """
        装饰器：打印函数执行时间，debug 级别输出。
        使用：@logger.measure_time
        """

        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start = time.time()
            try:
                return func(*args, **kwargs)
            finally:
                elapsed_ms = (time.time() - start) * 1000
                msg = (
                    f"{LogColors.CYAN}{func.__name__}{LogColors.BLUE} "
                    f"执行时间: {elapsed_ms:.2f} ms"
                )
                # 初始化检查
                self._check_initialized()
                # 直接调用底层 logger，stacklevel=3 跳过 wrapper 和 measure_time 两层
                self.logger.debug(msg, stacklevel=3)

        return wrapper

    # ===== 链式计时 =====
    class _ChainTimer:
        """内部类：链式计时器，可调用 / 链式 / with"""

        def __init__(self, outer: "Logger", level: int) -> None:
            self._outer = outer  # 外部 Logger 实例
            self._logger = outer.logger  # 底层 logging.Logger
            self._level = level
            self._t0 = time.perf_counter()  # 起点
            self._last = self._t0  # 上一次 checkpoint

        # -- 把对象本身做成“可调用”，用起来就像函数 ----------------
        def __call__(self, name: str | None = None):
            now = time.perf_counter()
            span_ms = (now - self._last) * 1000
            total_ms = (now - self._t0) * 1000
            msg = f"{name or '检查点'}: {span_ms:.2f} ms, (total: {total_ms:.2f} ms)"
            # stacklevel=3 → 日志显示调用点行号
            self._logger.log(self._level, msg, stacklevel=3)
            self._last = now
            return self  # 支持链式：t("A")("B")

        # -- 主动结束 --------------------------------------------------
        def done(self, name: str | None = None):
            self(name or "结束")  # 复用 __call__
            return None

        # -- with 协议支持 --------------------------------------------
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc_val, exc_tb):
            self.done()  # 自动收尾

    # 对外工厂方法 -----------------------------------------------------
    def timer(self, *, level: int = logging.DEBUG) -> "_ChainTimer":
        """
        返回一个链式计时器：
        # ① 链式调用 ---------------------------------------------------------
        t = logger.timer(level=logging.DEBUG)      # 开始计时
        ... heavy_load()
        t("加载完毕")                                     # + 时间差 / 累计
        ... train()
        t("训练完毕")                                     # + 时间差 / 累计
        ... infer()
        t.done("全部结束")                                # 结束（可选）

        # ② 可调用对象 + with，上下文自动收尾 ------------------------------
        with logger.timer(level=logging.INFO) as c:  # 进入即开始
            do_a()
            c("阶段 A")                                    # checkpoint
            do_b()
            c("阶段 B")
        # 退出 with 时会自动再 log 一条 “结束” —— 可配合长任务
        """
        return self._ChainTimer(self, level)


# 全局logger
logger = Logger()

# 测试logger
# -----------------------------------------------------------
if __name__ == '__main__':
    # 使用示例
    logger.debug("这是一条DEBUG信息")
    logger.info("这是一条INFO信息")
    logger.warning("这是一条WARNING信息")
    logger.error("这是一条ERROR信息")
    logger.critical("这是一条CRITICAL信息")

    logger.init_logger(name='new_logger', level=logging.DEBUG, save_path="./test.log", detailed=False)


    @logger.measure_time
    def function_a():
        logger.debug("这是来自function_a的信息")


    def function_b():
        d = {'a': 1, 'b': 2, 'c': {'aaaaaaaaa': 1, 'bbbbbbbbbbbb': 2, 'ccccccccccc': 3, 'ddddddddddddddd': 4}}
        logger.debug("这是来自function_b的信息 %s", d)


    t = logger.timer()
    function_a()
    t()
    function_b()
    t.done()
