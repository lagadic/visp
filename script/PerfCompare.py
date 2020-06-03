from __future__ import print_function
from __future__ import division

import argparse
from enum import Enum
import xml.etree.ElementTree as ET
from collections import OrderedDict

def nanoToMilli(nano):
    return nano / 1000

class BenchmarkResult:
    """BenchmarkResult class to hold perf numbers"""

    def __init__(self, name, mean_value, mean_lower_bound, mean_upper_bound, \
                 std_val, std_lower_bound, std_upper_bound):
        self.name = name
        self.mean_value = nanoToMilli(mean_value)
        self.mean_lower_bound = nanoToMilli(mean_lower_bound)
        self.mean_upper_bound = nanoToMilli(mean_upper_bound)
        self.std_val = nanoToMilli(std_val)
        self.std_lower_bound = nanoToMilli(std_lower_bound)
        self.std_upper_bound = nanoToMilli(std_upper_bound)

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

parser = argparse.ArgumentParser()
parser.add_argument("--before", help="Path to XML perf log for before comparison.", required=True)
parser.add_argument("--after", help="Path to XML perf log for after comparison.", required=True)
parser.add_argument("--before-label", help="Label for before column.", default='Before')
parser.add_argument("--after-label", help="Label for after column.", default='After')
parser.add_argument("--metric", help="Benchmark metric (mean, low_mean, high_mean).", type=Metric, choices=list(Metric), default=Metric.mean)

args = parser.parse_args()

file_before = args.before
file_after = args.after
print('Path to XML perf log for before comparison:', file_before)
print('Path to XML perf log for after comparison:', file_after)
print()

tree_before = ET.parse(file_before)
tree_after = ET.parse(file_after)

root_before = tree_before.getroot()
root_after = tree_after.getroot()

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

results_before = loadResults(root_before)
results_after = loadResults(root_after)

for r_before_name, r_before in results_before.items():
    print('| {} | {} | {} | Speed-up |'.format(r_before_name, args.before_label, args.after_label))
    print('| --- | --- | --- | --- |')
    if r_before_name in results_after:
        r_after = results_after[r_before_name]

        for b_before_name, b_before in r_before.results.items():
            if b_before_name in r_after.results:
                b_after = r_after.results[b_before_name]

                metric_before = b_before.mean_value
                metric_after = b_after.mean_value

                if args.metric == Metric.lowMean:
                    metric_before = b_before.mean_lower_bound
                    metric_after = b_after.mean_lower_bound
                elif args.metric == Metric.highMean:
                    metric_before = b_before.mean_upper_bound
                    metric_after = b_after.mean_upper_bound

                speed_up = metric_before / metric_after
                print('| {} | {:.2f} | {:.2f} | {:.2f} |'.format(b_before_name, metric_before, metric_after, speed_up))
        print()
