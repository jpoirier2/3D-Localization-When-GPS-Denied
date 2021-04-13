import ast
import numpy as np
import string

FREQ = 96e6

def readTXT():
    """
    Reads from data.txt, evals each line as a variable, then crams that all into a list
    Returns that list, where each index contains a dictionary of data received from each node
    If you receive a list with more or less than four indices, SOMETHING IS VERY WRONG
    (or we actually made the system modular)
    """
    with open('data.txt', 'r') as d:
        dataList = []
        for l in d:
            dataList.append(ast.literal_eval(l))


    return dataList

def getTimestamps(data: list):
    """
    Accepts a list of dictionaries containing data from the nodes
    Extracts the timestamp from each dictionary and places it in a list that
    is arranged in alphabetical order according to the node's ID
    """
    timestampArray = np.empty(len(data))
    for d in data:
        timestampArray[string.ascii_uppercase.index(d['ID'])] = d['timestamp']
    return timestampArray
