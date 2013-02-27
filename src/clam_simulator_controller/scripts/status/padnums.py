#coding: UTF8
"""
Prints out a table, padded to make it pretty.

call pprint_table with an output (e.g. sys.stdout, cStringIO, file)
and table as a list of lists. Make sure table is "rectangular" -- each
row has the same number of columns.

MIT License
"""

__version__ = "0.1"
__author__ = "Ryan Ginstrom"

import locale
locale.setlocale(locale.LC_NUMERIC, "")

def format_num(num):
    """Format a number according to given places.
    Adds commas, etc.
    
    Will truncate floats into ints!"""

    try:
        #inum = int(num)
        #return locale.format("%.*f", (0, inum), True)
        stringy = str(num)
        if(len(stringy) > 11):
            return stringy[:(-1*(len(stringy) - 11))]
        else:
            return stringy

    except (ValueError, TypeError):
        return str(num)

def get_max_width(table, index):
    """Get the maximum width of the given column index
    """
    
    return max([len(format_num(row[index])) for row in table])

def pprint_table(out, table):
    """Prints out a table of data, padded for alignment
    
    @param out: Output stream ("file-like object")
    @param table: The table to print. A list of lists. Each row must have the same
    number of columns.
    
    """

    col_paddings = []
    
    for i in range(len(table[0])):
        col_paddings.append(get_max_width(table, i))

    for row in table:
        # left col
        print >> out, row[0].ljust(col_paddings[0] + 1),
        # rest of the cols
        for i in range(1, len(row)):
            col = format_num(row[i]).rjust(col_paddings[i] + 2)
            print >> out, col,
        print >> out

if __name__ == "__main__":
    table = [["", "taste", "land speed", "life"],
        ["spam", 300101, 4, 1003],
        ["eggs", 105, 13, 42],
        ["lumberjacks", 13, 105, 10]]
    
    import sys
    out = sys.stdout
    pprint_table(out, table)
