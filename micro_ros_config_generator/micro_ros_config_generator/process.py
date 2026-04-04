import jinja2
import xacro
import xml.etree.ElementTree as ET

doc = xacro.process_file("robot.xacro")
urdf_xml = doc.toxml()

root = ET.fromstring(urdf_xml)



env = jinja2.Environment(loader = jinja2.FileSystemLoader('.'))

template = env.get_template('template.txt')

actuators = []
for joint in root.findall('actuator'):
    actuator = {'virtual_pin': 0, 'physical_pin': 90, 'servo_type': 'SERVO_TYPE_180', 'security_min_angle': 0, 'security_max_angle': 270, 'default_angle': 135, 'init_joint': 5, 'move_joint': 5}
    actuator['virtual_pin'] = int(joint.find('hardware').find('pin').get('value'))
    actuator['physical_pin'] = int(joint.find('hardware').find('pin').get('value'))
    actuator['servo_type'] = joint.find('hardware').find('type').get('value')
    actuator['security_min_angle'] = int(joint.find('hardware').find('limit_lower').get('value'))
    actuator['security_max_angle'] = int(joint.find('hardware').find('limit_upper').get('value'))
    actuator['default_angle'] = int(joint.find('hardware').find('default_position').get('value'))
    actuators.append(actuator)
    

context = {
    'actuators': actuators
}
    

res = template.render(context)
print(res)
