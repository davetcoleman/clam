from lxml import etree
import shlex
import sys
import io

def doRound(values,decimal_places):
    num_vector = shlex.split(values)
    new_vector = []

    for num in num_vector:
        new_num = round(float(num),decimal_places)
        print "Old:",num,"New:",new_num
        new_vector.append(str(new_num))

    new = " ".join(new_vector)
    #print 'Original:', values, '  Updated: ', new

    return new

# -----------------------------------------------------------------------------

if __name__ == '__main__':

    if len(sys.argv) != 4:
        print ''
        print 'Usage: round_collada_numbers.py <input_dae> <output_dae> <decimal places>'
        print 'Rounds all the numbers to <decimal places> places\n'
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    decimal_places = int(sys.argv[3])

    print 'Rounding numbers to', decimal_places, ' decimal places\n'

    # Read string from file
    f = open(input_file,'r')
    xml = f.read();

    # Parse XML
    #doc = etree.fromstring(xml)
    #print(doc.tag)
    #doc = etree.parse(io.BytesIO(xml))
    #element=doc.xpath('//ns:asset',namespaces={'ns','http://www.collada.org/2008/03/COLLADASchema'})
    #print element

    namespace = 'http://www.collada.org/2008/03/COLLADASchema'
    dom = etree.parse(io.BytesIO(xml))

    # find elements of particular name
    elements=dom.xpath('//ns:translate',namespaces={'ns':namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text,decimal_places)

    # find elements of particular name
    elements=dom.xpath('//ns:rotate',namespaces={'ns':namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text,decimal_places)

    # find elements of particular name
    elements=dom.xpath('//ns:min',namespaces={'ns':namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text,decimal_places)

    # find elements of particular name
    elements=dom.xpath('//ns:max',namespaces={'ns':namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text,decimal_places)

    # find elements of particular name
    elements=dom.xpath('//ns:float',namespaces={'ns':namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text,decimal_places)

    # save changes
    f = open(output_file,'w')
    f.write(etree.tostring(dom))
    f.close()


