import os


def __init__():
    default_text1 = """\"<robot name=\"_name\">
        <link name=\"_name\">
            <visual>
                <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
                <geometry>
                    <mesh filename=\""""

    default_text2 = """.obj\" scale=\"1.0 1.0 1.0\" />
                </geometry>
                <material name=\"texture\">
                    <color rgba=\"1.0 1.0 1.0 1.0\"/>
                </material>
            </visual>
            <collision>
                <geometry>
                    <mesh filename=\""""
    default_text3 = """.obj\" scale=\"1.0 1.0 1.0\" />
                </geometry>
            </collision>
            <inertial>
                <mass value=\"1.0\"/>
                <inertia ixx=\"1.0\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1.0\" iyz=\"0.0\" izz=\"1.0\"/>
            </inertial>
        </link>
    </robot>"""

    for filename in os.listdir("./objects"):
        if filename[-5:] != ".urdf":
            obj = filename
            obj += "/"
            obj += filename
            urdf = "objects/"+filename
            urdf += ".urdf"

            f = open(urdf, "w")
            f.write(default_text1)
            f.write(obj)
            f.write(default_text2)
            f.write(obj)
            f.write(default_text3)
            f.close()
