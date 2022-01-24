from __future__ import print_function
from __future__ import division

import argparse
from enum import Enum
import xml.etree.ElementTree as ET
from collections import OrderedDict
import matplotlib
import matplotlib.pyplot as plt

class BenchmarkResult:
    """BenchmarkResult class to hold perf numbers"""

    def __init__(self, name, mean_value, mean_lower_bound, mean_upper_bound, \
                 std_val, std_lower_bound, std_upper_bound):
        self.name = name
        self.mean_value = mean_value
        self.mean_lower_bound = mean_lower_bound
        self.mean_upper_bound = mean_upper_bound
        self.std_val = std_val
        self.std_lower_bound = std_lower_bound
        self.std_upper_bound = std_upper_bound

    def __str__(self):
        return "BenchmarkResults %s\nmean value=%f, lowerBound=%f, upperBound=%f\nstd:%f, lowerBound=%f, upperBound=%f" \
                % (self.name, self.mean_value, self.mean_lower_bound, self.mean_upper_bound, \
                   self.std_val, self.std_lower_bound, self.std_upper_bound)

class TestCase:
    """TestCase class to hold the list of benchmark results"""

    def __init__(self, name):
        self.name = name
        self.results = OrderedDict()

    def __str__(self):
        return "TestCase %s\nBenchmark result:\n%s" % (self.name, self.results)

class SectionCase:
    """SectionCase class to hold the section list of benchmark results"""

    def __init__(self, name):
        self.name = name
        self.results = OrderedDict()

    def __str__(self):
        return "SectionCase %s\nBenchmark result:\n%s" % (self.name, self.results)

class Metric(Enum):
    mean = 'mean'
    lowMean = 'low_mean'
    highMean = 'high_mean'

class Unit(Enum):
    nano = 'nano'
    micro = 'micro'
    milli = 'milli'
    sec = 'sec'

def displayUnit(unit):
    if unit == Unit.sec:
        return "s"
    elif unit == Unit.milli:
        return "ms"
    elif unit == Unit.micro:
        return "us"
    else:
        return "ns"

def nanoToMicro(nano):
    return nano / 1000

def nanoToMilli(nano):
    return nano / (1000*1000)
    # return nano

def nanoToSec(nano):
    return nano / (1000*1000*1000)

def convertUnit(nano, unit):
    if unit == Unit.sec:
        return nanoToSec(nano)
    elif unit == Unit.milli:
        return nanoToMilli(nano)
    elif unit == Unit.micro:
        return nanoToMicro(nano)
    else:
        return nano

parser = argparse.ArgumentParser()
parser.add_argument("--xml-file", help="Path to XML perf log.", required=True)
parser.add_argument("--label", help="Label for before column.", default='Before')
parser.add_argument("--metric", help="Benchmark metric (mean, low_mean, high_mean).", type=Metric, choices=list(Metric), default=Metric.mean)
parser.add_argument("--unit", help="Benchmark unit (nano, micro, milli, sec).", type=Unit, choices=list(Unit), default=Unit.milli)

args = parser.parse_args()

xml_file = args.xml_file
metric = args.metric
time_unit = args.unit

print("Matplotlib: {}".format(matplotlib.__version__))
print('Path to XML log file for perf comparison:', xml_file)
print('Perf metric:', metric)
print('Time unit:', time_unit)
print()

tree_perf_data = ET.parse(xml_file)

root_perf_data = tree_perf_data.getroot()

def loadResults(root):
    results = OrderedDict()

    for test_case in root.iter('TestCase'):
        test_name = test_case.attrib['name']
        current_test = TestCase(test_name)

        for section_case in test_case.iter('Section'):
            case_name = section_case.attrib['name']
            current_section = SectionCase(case_name)

            for bench_res in section_case.iter('BenchmarkResults'):
                bench_name = bench_res.attrib['name']

                mean_node = bench_res.find('mean')
                mean = float(mean_node.attrib['value'])
                mean_lower = float(mean_node.attrib['lowerBound'])
                mean_upper = float(mean_node.attrib['upperBound'])

                std_node = bench_res.find('standardDeviation')
                std = float(std_node.attrib['value'])
                std_lower = float(std_node.attrib['lowerBound'])
                std_upper = float(std_node.attrib['upperBound'])

                current_section.results[bench_name] = BenchmarkResult(bench_name, mean, mean_lower, mean_upper, std, std_lower, std_upper)

            current_test.results[case_name] = current_section

        results[test_name] = current_test

    return results

results_perf_data = loadResults(root_perf_data)

for r_name, r_test in results_perf_data.items():
    fig, axs = plt.subplots(ncols=len(r_test.results.items()))
    idx1 = 0

    for r_section_name, r_section in r_test.results.items():
        x_list = []
        y_list = []
        std_list = []
        idx2 = 0
        backend_list = []

        for r_result_name, r_result in r_section.results.items():
            x_list.append(idx2)
            if metric == Metric.lowMean:
                y_list.append(convertUnit(r_result.mean_lower_bound, time_unit))
            elif metric == Metric.highMean:
                y_list.append(convertUnit(r_result.mean_higher_bound, time_unit))
            else:
                y_list.append(convertUnit(r_result.mean_value, time_unit))
            std_list.append(convertUnit(r_result.std_val, time_unit))
            backend_list.append(r_result_name.replace(" backend", ""))
            idx2 += 1

        axs[idx1].bar(x_list, y_list, yerr=std_list, align='center', capsize=10, tick_label=backend_list)
        axs[idx1].grid(True)
        axs[idx1].xaxis.set_tick_params(labelsize=14)
        axs[idx1].set_xlabel(r_section_name, fontsize=16)
        axs[0].set_ylabel("Computation time (ms)", fontsize=16)

        plt.setp(axs[idx1].get_xticklabels(), rotation=45)
        idx1 += 1

    plt.suptitle(r_name, fontsize=24)
    plt.show()
