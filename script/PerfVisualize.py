from __future__ import print_function
from __future__ import division

import argparse
from enum import Enum
import xml.etree.ElementTree as ET
from collections import OrderedDict

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
    # return nano / (1000*1000)
    return nano

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
parser.add_argument("--unit", help="Benchmark unit (nano, micro, milli, sec).", type=Unit, choices=list(Unit), default=Unit.micro)

args = parser.parse_args()

xml_file = args.xml_file
unit = args.unit
print('Path to XML log file for perf comparison:', xml_file)
print('Time unit:', unit)
print()

tree_perf_data = ET.parse(xml_file)

root_perf_data = tree_perf_data.getroot()

def loadResults(root):
    results = OrderedDict()

    for test_case in root.iter('TestCase'):
        test_name = test_case.attrib['name']

        current_test = TestCase(test_name)

        for bench_res in test_case.iter('BenchmarkResults'):
            bench_name = bench_res.attrib['name']

            mean_node = bench_res.find('mean')
            mean = float(mean_node.attrib['value'])
            mean_lower = float(mean_node.attrib['lowerBound'])
            mean_upper = float(mean_node.attrib['upperBound'])

            std_node = bench_res.find('standardDeviation')
            std = float(std_node.attrib['value'])
            std_lower = float(std_node.attrib['lowerBound'])
            std_upper = float(std_node.attrib['upperBound'])

            current_test.results[bench_name] = BenchmarkResult(bench_name, mean, mean_lower, mean_upper, std, std_lower, std_upper)

        results[test_name] = current_test

    return results

results_perf_data = loadResults(root_perf_data)

print("results_perf_data:\n", results_perf_data)

# for r_before_name, r_before in results_before.items():
#     title = '{} - time unit: {}'.format(r_before_name, displayUnit(unit))
#     print('| {} | {} | {} | Speed-up |'.format(title, args.before_label, args.after_label))
#     print('| --- | --- | --- | --- |')
#     if r_before_name in results_after:
#         r_after = results_after[r_before_name]

#         for b_before_name, b_before in r_before.results.items():
#             if b_before_name in r_after.results:
#                 b_after = r_after.results[b_before_name]

#                 if args.metric == Metric.lowMean:
#                     metric_before = b_before.mean_lower_bound
#                     metric_after = b_after.mean_lower_bound
#                 elif args.metric == Metric.highMean:
#                     metric_before = b_before.mean_upper_bound
#                     metric_after = b_after.mean_upper_bound
#                 else:
#                     metric_before = b_before.mean_value
#                     metric_after = b_after.mean_value

#                 metric_before = convertUnit(metric_before, unit)
#                 metric_after = convertUnit(metric_after, unit)

#                 speed_up = metric_before / metric_after
#                 print('| {} | {:.2f} | {:.2f} | {:.2f} |'.format(b_before_name, metric_before, metric_after, speed_up))
#         print()
