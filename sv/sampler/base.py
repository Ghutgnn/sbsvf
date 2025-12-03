# sampler.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Iterable, Dict, Any, List
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import logging

from sv.registry import register_sampler

logger = logging.getLogger(__name__)

ParamDict = Dict[str, Any]
TestResult = Dict[str, Any]


class Sampler:
    """
    非強制繼承，只是當作型別說明用。
    你也可以改成 Protocol，不影響下面設計。
    """

    def next(
        self,
        past_results: Optional[Iterable[TestResult]] = None,
    ) -> Optional[ParamDict]:
        raise NotImplementedError


@dataclass
class ParameterSpec:
    name: str
    values: List[float]


def frange_inclusive(
    lower: float, upper: float, step: float, tol: float = 1e-9
) -> List[float]:
    if step <= 0:
        raise ValueError("step must be positive")

    n_steps = int(math.floor((upper - lower) / step + tol))
    vals = []
    for i in range(n_steps + 1):
        v = lower + i * step
        if v > upper + tol:
            break
        vals.append(v)

    # if vals and upper - vals[-1] > tol:
    #     vals.append(upper)

    return vals


def parse_parameter_value_distribution(xml_str: str) -> List[ParameterSpec]:
    root = ET.fromstring(xml_str)
    pvd = (
        root
        if root.tag == "ParameterValueDistribution"
        else root.find(".//ParameterValueDistribution")
    )
    if pvd is None:
        raise ValueError("Cannot find ParameterValueDistribution element")

    det = pvd.find("Deterministic")
    if det is None:
        raise ValueError("Only <Deterministic> distributions are supported for now")

    specs: List[ParameterSpec] = []

    for elem in det.findall("DeterministicSingleParameterDistribution"):
        name = elem.attrib["parameterName"]

        dist_range = elem.find("DistributionRange")
        if dist_range is None:
            raise ValueError(f"Missing <DistributionRange> for parameter {name}")

        step = float(dist_range.attrib["stepWidth"])

        range_elem = dist_range.find("Range")
        if range_elem is None:
            raise ValueError(f"Missing <Range> for parameter {name}")

        lower = float(range_elem.attrib["lowerLimit"])
        upper = float(range_elem.attrib["upperLimit"])

        values = frange_inclusive(lower, upper, step)
        specs.append(ParameterSpec(name=name, values=values))

    return specs


class BaseSampler(Sampler):
    def __init__(self, specs: List[ParameterSpec]):
        self.specs = specs

    def update_with_results(self, past_results: Optional[Iterable[TestResult]]):
        if not past_results:
            return
        # 這裡之後可以放共用的統計 / logging
