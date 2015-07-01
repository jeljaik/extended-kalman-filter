#!/usr/bin/python

import argparse

def printPortTags(ports, port_type, host):
    for port in ports:
        # print datadumper launch 
        print("\t<module>");
        print("\t\t<name>dataDumper</name>");
        print("\t\t<parameters>--name /dumper"+port+" --type "+port_type+"</parameters>");
        print("\t\t<node>"+host+"</node>");
        print("\t\t<tag>data-dumper"+port.replace('/','-').replace(':','-')+"</tag>"); 
        print("\t</module>");
 
        # print connection between datadumper and port
        print("\t<connection>");
        print("\t\t<from>"+port+"</from>");
        print("\t\t<to>/dumper"+port+"</to>");
        print("\t\t<protocol>udp</protocol>");
        print("\t</connection>");

def main():
    parser = argparse.ArgumentParser(description='Tool for generating a YarpManager XML application for dumping a list of YARP ports using the dataDumper.')
    parser.add_argument('--ports', nargs='+', dest="ports", action='store', required=True, help='list of ports (serializable to bottles) to dump')
    parser.add_argument('--imagePorts', nargs='+', dest="imagePorts", action='store', help='list of ports (of to dump')
    parser.add_argument('--host', nargs=1,  dest="host", action='store', required=True, help='host where to launch the dataDumpers')
    parser.add_argument('--name', nargs=1,  dest="name", action='store', required=True, help='name of the application')
    args = parser.parse_args()

    print("<application>");
    print("\t<name>"+args.name[0]+"</name>");
    print("\t<dependencies>");
    for port in args.ports:
        print("\t\t<port>"+port+"</port>");
    print("\t</dependencies>");

    if( args.ports is not None):
        printPortTags(args.ports,"bottle",args.host[0]);
    if( args.imagePorts is not None):
        printPortTags(args.imagePorts,"image",args.host[0]);
    
    print("</application>");
  
if __name__ == "__main__":
    main()