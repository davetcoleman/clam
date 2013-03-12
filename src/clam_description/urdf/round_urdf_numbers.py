from lxml import etree
import shlex
import sys

def doRound(values,decimal_places):
    num_vector = shlex.split(values)
    new_vector = []

    for num in num_vector:
        new_num = round(float(num),decimal_places)
        #print "Old:",num,"New:",new_num
        new_vector.append(str(new_num))

    new = " ".join(new_vector)

    print 'Original:', values, '  Updated: ', new
    return new

# -----------------------------------------------------------------------------

if __name__ == '__main__':

    if len(sys.argv) != 4:
        print ''
        print 'Usage: round_urdf_numbers.py <input_urdf> <output_urdf> <decimal places>'
        print 'Rounds all the numbers to <decimal places> places for use in converting to Collada\n'
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    decimal_places = int(sys.argv[3])

    print 'Rounding numbers to', decimal_places, ' decimal places\n'

    # Read string from file
    f = open(input_file,'r')
    xml = f.read();

    # Parse XML
    doc = etree.fromstring(xml)

    # origin.rpy, origin.xyz
    for origin in  doc.iter('origin'):
        if 'rpy' in origin.attrib:
            origin.attrib["rpy"] = doRound(origin.attrib["rpy"],decimal_places)
        if 'xyz' in origin.attrib:
            origin.attrib["xyz"] = doRound(origin.attrib["xyz"],decimal_places)

    # axis.xyz
    for axis in doc.iter('axis'):
        if 'xyz' in axis.attrib:
            axis.attrib["xyz"] = doRound(axis.attrib["xyz"],decimal_places)

    # limit.lower, limit.upper
    for limit in  doc.iter('limit'):
        if 'lower' in limit.attrib:
            limit.attrib["lower"] = doRound(limit.attrib["lower"],decimal_places)
        if 'upper' in limit.attrib:
            limit.attrib["upper"] = doRound(limit.attrib["upper"],decimal_places)

    # save changes
    f = open(output_file,'w')
    f.write(etree.tostring(doc))
    f.close()


