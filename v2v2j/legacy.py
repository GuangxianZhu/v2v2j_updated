"""原版算法（2023 年手写版，见 legacy/v2v2j.ipynb）的忠实移植——只删掉了调试 print，逻辑一字未改。

保留它的目的：
1. 对照实验：同一个网表，原版 vs 新版，用同一个校验器和仿真器评判；
2. 教学：教学网页里的"原版为什么错"就是跑这份代码得到的。

已知问题（均有测试用例复现，见 tests/test_legacy_bugs.py）：
- assign_levels 用 DFS "首次访问即定级"，得到的是某条路径的深度而不是最长路径；
- add_buffers 只遍历单元输出，主输入直连深层门时不会补 buffer；
- add_splitters 只统计单元输入脚，漏了主输出这个负载，也不做输出相位对齐；
- 扇出 > 4 直接报错；正则解析不认总线 / 转义名 / assign。
"""
# fmt: off
# flake8: noqa
import re

def parse_library(lib_path):
    # Initialize the final data structure
    lib_data = {}

    with open(lib_path, 'r') as f_lib:
        library = f_lib.read()

        # Extract module details
        modules = re.findall(r"module (\w+?)\((.*?)\);(.*?)endmodule", library, re.S)

        for module in modules:
            module_name, ports, content = module
            lib_data[module_name] = {
                'inputs': [],
                'outputs': [],
                'inouts': [],
            }

            # Parse inputs, outputs and inouts
            inputs = re.findall(r"input (.*?);", content)
            outputs = re.findall(r"output (.*?);", content)
            inouts = re.findall(r"inout (.*?);", content)

            for inp in inputs:
                lib_data[module_name]['inputs'].extend([x.strip() for x in inp.split(",")])

            for out in outputs:
                lib_data[module_name]['outputs'].extend([x.strip() for x in out.split(",")])

            for inout in inouts:
                lib_data[module_name]['inouts'].extend([x.strip() for x in inout.split(",")])

    return lib_data

def parse_netlist(netlist_path, lib_data):
    # Initialize the final data structure
    data = {}

    with open(netlist_path, 'r') as f_netlist:
        netlist = f_netlist.read()

        # Extract module details
        modules = re.findall(r"module (\w+?)\((.*?)\);(.*?)endmodule", netlist, re.S)

        for module in modules:
            module_name, ports, content = module
            # print('module_name', module_name, 'ports', ports, 'content', content)
            data[module_name] = {
                'inputs': [],
                'outputs': [],
                'wires': [],
                'cells': [],
            }

            # Parse inputs, outputs and wires
            inputs = re.findall(r"input (.*?);", content)
            outputs = re.findall(r"output (.*?);", content)
            wires = re.findall(r"wire (.*?);", content)

            for inp in inputs:
                data[module_name]['inputs'].extend([x.strip() for x in inp.split(",")])

            for out in outputs:
                data[module_name]['outputs'].extend([x.strip() for x in out.split(",")])

            for wire in wires:
                data[module_name]['wires'].extend([x.strip() for x in wire.split(",")])

            # Parse cell details
            cells = re.findall(r"(\w+) (\w+?) \((.*?)\);", content, re.S)
            # print('cells', cells)

            for cell in cells:
                # print('cell', cell)
                cell_type, cell_name, pin_info = cell
                # print('cell_type', cell_type, 'cell_name', cell_name, 'pin_info', pin_info)
                data[module_name]['cells'].append({
                    'cell_name': cell_name,
                    'cell_type': cell_type,
                    'pins': [],
                })

                pins = re.findall(r"\.(.*?)\((.*?)\)", pin_info)
                for pin in pins:
                    pin_name, pin_wire = pin[0].strip(), pin[1].strip()
                    pin_type = 'input' if pin_name in lib_data[cell_type]['inputs'] else \
                                'output' if pin_name in lib_data[cell_type]['outputs'] else \
                                'inout' if pin_name in lib_data[cell_type]['inouts'] else 'unknown'
                    data[module_name]['cells'][-1]['pins'].append({
                        'pin_name': pin_name,
                        'pin_type': pin_type,
                        'pin_wire': pin_wire,
                    })
    return data

def print_data(data):
    for module, module_data in data.items():
        print(f"Module: {module}\n")
        print("Inputs:")
        for inp in module_data['inputs']:
            print(f"  {inp}")
        print("\nOutputs:")
        for out in module_data['outputs']:
            print(f"  {out}")
        print("\nWires:")
        for wire in module_data['wires']:
            print(f"  {wire}")
        print("\nCells:")
        for cell in module_data['cells']:
            print(f"  Cell Name: {cell['cell_name']}")
            print(f"  Cell Type: {cell['cell_type']}")
            print("  Pins:")
            for pin in cell['pins']:
                print(f"    Pin Name: {pin['pin_name']}")
                print(f"    Pin Type: {pin['pin_type']}")
                print(f"    Connected to: {pin['pin_wire']}")
            print()
        print("-------\n")

def write_data_to_file(data, file_path):
    with open(file_path, 'w') as f:
        for module, module_data in data.items():
            f.write(f"Module: {module}\n\n")
            f.write("Inputs:\n")
            for inp in module_data['inputs']:
                f.write(f"  {inp}\n")
            f.write("\nOutputs:\n")
            for out in module_data['outputs']:
                f.write(f"  {out}\n")
            f.write("\nWires:\n")
            for wire in module_data['wires']:
                f.write(f"  {wire}\n")
            f.write("\nCells:\n")
            for cell in module_data['cells']:
                f.write(f"  Cell Name: {cell['cell_name']}\n")
                f.write(f"  Cell Type: {cell['cell_type']}\n")
                f.write("  Pins:\n")
                for pin in cell['pins']:
                    f.write(f"    Pin Name: {pin['pin_name']}\n")
                    f.write(f"    Pin Type: {pin['pin_type']}\n")
                    f.write(f"    Connected to: {pin['pin_wire']}\n")
                f.write("\n")
            f.write("-------\n")


def initialize_levels(data):
    for module_data in data.values():
        for cell in module_data['cells']:
            cell['level'] = 0

def assign_levels(data):
    def helper(wire, level):
        for cell in module_data['cells']:
            if cell['level'] == 0:  # only update the level if it hasn't been assigned yet
                for pin in cell['pins']:
                    if pin['pin_wire'] == wire and pin['pin_type'] == 'input':
                        cell['level'] = level
                        for pin_out in cell['pins']:
                            if pin_out['pin_type'] == 'output':
                                helper(pin_out['pin_wire'], level + 1)

    for module_data in data.values():
        for wire in module_data['inputs']:
            helper(wire, 1)

def print_data_with_levels(data):
    for module, module_data in data.items():
        print(f"Module: {module}\n")
        print("Inputs:")
        for inp in module_data['inputs']:
            print(f"  {inp}")
        print("\nOutputs:")
        for out in module_data['outputs']:
            print(f"  {out}")
        print("\nWires:")
        for wire in module_data['wires']:
            print(f"  {wire}")
        print("\nCells:")
        for cell in module_data['cells']:
            print(f"  Cell Name: {cell['cell_name']}")
            print(f"  Cell Type: {cell['cell_type']}")
            print(f"  Cell Level: {cell['level']}")
            print("  Pins:")
            for pin in cell['pins']:
                print(f"    Pin Name: {pin['pin_name']}")
                print(f"    Pin Type: {pin['pin_type']}")
                print(f"    Connected to: {pin['pin_wire']}")
            print()
        print("-------\n")


def add_splitters(data, lib_data):
    for module_data in data.values():
        cell_connections = {}
        for cell in module_data['cells']:
            for pin in cell['pins']:
                if pin['pin_type'] == 'input':  # Use input pins here
                    if pin['pin_wire'] in cell_connections:  # if the wire is already in the dictionary
                        cell_connections[pin['pin_wire']].append(cell['cell_name'])
                    else:
                        cell_connections[pin['pin_wire']] = [cell['cell_name']]
        
        for wire, connected_cells in cell_connections.items():
            num_connected_cells = len(connected_cells)
            if num_connected_cells > 1:
                if num_connected_cells > 4:
                    raise ValueError("More than 4 cells are connected to the same wire. No suitable splitter defined.")
                else:
                    # Add a splitter
                    splitter_module_name = "spl" + str(num_connected_cells)
                    if splitter_module_name not in lib_data:
                        raise ValueError(f"The required splitter module {splitter_module_name} is not defined in the library.")
                    
                    splitter_cell_name = "splitter_" + wire  # name the splitter cell based on the wire it splits
                    splitter = {
                        'cell_name': splitter_cell_name,
                        'cell_type': splitter_module_name,
                        'pins': [],
                        'level': 0  # initialize the splitter's level to 0
                    }
                    splitter_inputs = lib_data[splitter_module_name]['inputs']
                    splitter_outputs = lib_data[splitter_module_name]['outputs']
                    
                    # Connect the splitter's input to the wire
                    splitter['pins'].append({
                        'pin_name': splitter_inputs[0],
                        'pin_type': 'input',
                        'pin_wire': wire
                    })
                    
                    # Connect the splitter's outputs to the cells that were previously connected to the wire
                    for i, cell_name in enumerate(connected_cells):
                        splitter['pins'].append({
                            'pin_name': splitter_outputs[i],
                            'pin_type': 'output',
                            'pin_wire': wire + "_split_" + str(i)
                        })
                        # Update the wire connected to the input pin of the connected cell
                        for cell in module_data['cells']:
                            if cell['cell_name'] == cell_name:
                                for pin in cell['pins']:
                                    if pin['pin_type'] == 'input' and pin['pin_wire'] == wire:
                                        pin['pin_wire'] = wire + "_split_" + str(i)
                    
                    # Add the splitter cell to the cells list
                    module_data['cells'].append(splitter)

    return data


def add_buffers(data, lib_data):
    if 'bfr' not in lib_data:
        raise ValueError("Buffer module 'bfr' is not defined in the library.")
    
    buffer_inputs = lib_data['bfr']['inputs']
    buffer_outputs = lib_data['bfr']['outputs']
    
    for module_data in data.values():
        for cell in module_data['cells']:
            for pin in cell['pins']:
                if pin['pin_type'] == 'output':
                    connected_cells = [connected_cell for connected_cell in module_data['cells'] if any(connected_pin['pin_type'] == 'input' and connected_pin['pin_wire'] == pin['pin_wire'] for connected_pin in connected_cell['pins'])]
                    for connected_cell in connected_cells:
                        level_difference = connected_cell['level'] - cell['level']
                        if level_difference > 1:
                            previous_buffer_output_wire = pin['pin_wire']
                            for i in range(level_difference - 1):
                                buffer_cell_name = "buffer_" + previous_buffer_output_wire + "_" + str(i)
                                buffer = {
                                    'cell_name': buffer_cell_name,
                                    'cell_type': 'bfr',
                                    'pins': [],
                                    'level': cell['level'] + i + 1
                                }
                                
                                buffer['pins'].append({
                                    'pin_name': buffer_inputs[0],
                                    'pin_type': 'input',
                                    'pin_wire': previous_buffer_output_wire
                                })
                                
                                buffer_output_wire = previous_buffer_output_wire + "_buffer_" + str(i)
                                buffer['pins'].append({
                                    'pin_name': buffer_outputs[0],
                                    'pin_type': 'output',
                                    'pin_wire': buffer_output_wire
                                })
                                
                                module_data['cells'].append(buffer)
                                previous_buffer_output_wire = buffer_output_wire
                            
                            # Update the wire connected to the input pin of the connected cell
                            for connected_pin in connected_cell['pins']:
                                if connected_pin['pin_type'] == 'input' and connected_pin['pin_wire'] == pin['pin_wire']:
                                    connected_pin['pin_wire'] = previous_buffer_output_wire

    return data


# ---------------------------------------------------------------------------
# 以下为新增的适配层：把原版的 dict 结构转成新版 Netlist，以便用同一个校验器评判
def run_legacy(lib_path, netlist_text):
    """按原 notebook 的顺序跑一遍原版算法。返回原版的数据结构；失败时抛出原版的异常。"""
    import os, tempfile
    lib_data = parse_library(lib_path)
    with tempfile.NamedTemporaryFile("w", suffix=".v", delete=False) as f:
        f.write(netlist_text)
        path = f.name
    try:
        data = parse_netlist(path, lib_data)
    finally:
        os.unlink(path)
    data_spl = add_splitters(data, lib_data)
    initialize_levels(data_spl)
    assign_levels(data_spl)
    data_spl_bfr = add_buffers(data_spl, lib_data)
    initialize_levels(data_spl_bfr)
    assign_levels(data_spl_bfr)
    return data_spl_bfr


def legacy_to_netlist(data, lib):
    from .netlist import Inst, Netlist, Port
    (mname, m), = data.items()
    nl = Netlist(mname, lib)
    for i in m["inputs"]:
        nl.ports.append(Port(i, "input", [i]))
    for o in m["outputs"]:
        nl.ports.append(Port(o, "output", [o]))
    for c in m["cells"]:
        conns = {p["pin_name"]: p["pin_wire"] for p in c["pins"] if p["pin_type"] in ("input", "output")}
        org = {"bfr": "buffer"}.get(c["cell_type"], "splitter" if c["cell_type"].startswith("spl") else "logic")
        nl.add(Inst(c["cell_name"], c["cell_type"], conns, c.get("level"), org))
    return nl
