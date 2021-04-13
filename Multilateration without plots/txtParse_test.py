import txtParse as tp
import numpy as np

raw_data = tp.readTXT()
print(f'Raw data: {raw_data}')

cooked_data = tp.getTimestamps(raw_data)
print(f'Cooked data: {cooked_data}')

x = input('\n')
