import csv
import pandas
def parse1():
    with open('test.csv', 'r') as infile:
        reader = csv.reader(infile)
        with open('file_new.csv', 'w') as outfile:
            writer = csv.writer(outfile)
            for row in reader:
                new_row = [cell.replace("<", '').replace(">", '').replace('"', '') for cell in row]
                writer.writerow(new_row)

def readd():
    with open('file_new.csv', 'r') as file:
        reader =  csv.reader(file)
        for row in reader:
            print(row)

def readdd():
    df = pandas.read_csv('file_new.csv', usecols=['TeamCode'])
    print(df.head())


readdd()