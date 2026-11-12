#!/usr/bin/env python3
"""Export a URDF with absolute mesh paths for consumers without a ROS package index."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1],
                        help="Package share directory; set explicitly when using the installed script")
    args = parser.parse_args()
    package = args.package_dir.resolve()
    tree = ET.parse(package / "urdf/landerpi_mecanum.urdf")
    for mesh in tree.findall(".//mesh"):
        name = mesh.attrib["filename"]
        prefix = "package://landerpi_description/"
        if not name.startswith(prefix):
            raise ValueError(f"Unexpected mesh reference: {name}")
        path = package / name[len(prefix):]
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set("filename", str(path))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(args.output, encoding="utf-8", xml_declaration=True)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
