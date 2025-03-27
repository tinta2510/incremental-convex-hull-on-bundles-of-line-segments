from intervaltree import Interval, IntervalTree

tree = IntervalTree()
tree[0:4] = "Subsequence A"
tree[2:6] = "Subsequence B"

matches = tree[3]  # returns all subsequences that include index 3
print(list(matches)[-1].data)  # prints "Subsequence B"