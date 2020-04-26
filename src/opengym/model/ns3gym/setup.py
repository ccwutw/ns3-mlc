from setuptools import setup, find_packages
import sys
import os.path
import os
import re
import ns3gym

cwd = os.getcwd()
protobufFile = cwd + '/ns3gym/messages_pb2.py'

if not os.path.isfile(protobufFile):
    #if no protocbuf file, try to generate it
    rc = os.system('whereis protoc')
    #check whereis command
    if rc != 0:
        print("Command: ", "whereis protoc", "not found.")
        sys.exit("Missing command 'whereis', please check!")
    protoc = None
    #check protoc command
    rt = os.popen("whereis protoc").read()
    protoc_paths = re.finditer(r'/[^\s]+', rt)
    for protoc_path in protoc_paths:
        if 'conda' in protoc_path[0]:
            continue
        else:
            protoc = protoc_path[0]
            break
    # if no protoc command, then need to check Google Protocol Buffers install
    if protoc is None:
        print("File: ", "ns3-gym/src/opengym/model/ns3gym/ns3gym/messages_pb2.py", " was not found.")
        sys.exit('Protocol Buffer messages are missing. Please run check Google Protocol Buffers or /messages.proto file')
    #if has protoc command, try to generate python file
    print('generate proto file')
    os.chdir('../' )
    #input file = messages.proto
    os.system('protoc ./messages.proto --python_out=./ns3gym/ns3gym')
    os.chdir('./ns3gym')

def readme():
    with open('README.md') as f:
        return f.read()


setup(
    name='ns3gym',
    version='0.1.0',
    packages=find_packages(),
    scripts=[],
    url='',
    license='MIT',
    author='Piotr Gawlowicz',
    author_email='gawlowicz.p@gmail.com',
    description='OpenAI Gym meets ns-3',
    long_description='OpenAI Gym meets ns-3',
    keywords='openAI gym, ML, RL, ns-3',
    install_requires=['pyzmq', 'numpy', 'protobuf', 'gym'],
    extras_require={},
)
