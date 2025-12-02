from typing import Dict, Type, Any, Callable
import logging


logger = logging.getLogger(__name__)

SIM_REGISTRY: Dict[str, Any] = {}
AV_REGISTRY: Dict[str, Any] = {}
BRIDGE_REGISTRY: Dict[str, Any] = {}
MONITOR_REGISTRY: Dict[str, Any] = {}
MPACK_REGISTRY: Dict[str, Callable[[], Any]] = {}  # loader function


def _mk_register(table: Dict[str, Any]):
    def register(name: str):
        def deco(cls_or_fn):
            table[name] = cls_or_fn
            return cls_or_fn
        return deco
    return register


register_sim = _mk_register(SIM_REGISTRY)
register_av = _mk_register(AV_REGISTRY)
register_bridge = _mk_register(BRIDGE_REGISTRY)
register_monitor = _mk_register(MONITOR_REGISTRY)
register_mpack = _mk_register(MPACK_REGISTRY)


logger = logging.getLogger(__name__)


def build_instance_from_registry(
    registry: dict[str, any],
    name: str,
    **kwargs: any,
):
    """
    根據名稱從 registry 取出 class / callable。
    - 如果是 class：回傳 class(**kwargs)
    - 如果是 callable (工廠函式)：呼叫並回傳 callable(**kwargs)
    """
    if name not in registry:
        logger.error(
            f" '{name}' not found in registry. Available: {list(registry.keys())}")
        raise KeyError(
            f"{name!r} not found in registry. Available: {list(registry.keys())}")

    target = registry[name]
    logger.debug(
        f"Building instance for '{name}' from registry (type={type(target).__name__}) with args={kwargs}")

    if isinstance(target, type):
        instance = target(**kwargs)
        logger.info(f"Created instance of class '{target.__name__}'")
        return instance

    if callable(target):
        instance = target(**kwargs)
        logger.info(f"Created instance via callable '{target.__name__}'")
        return instance

    logger.error(
        f"Registered object '{name}' is not instantiable (type={type(target).__name__})")
    raise TypeError(f"Registered object {name!r} is not instantiable")
