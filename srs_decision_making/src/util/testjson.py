import json_parser

json_parameters = "{\"tasks\":[{\"time_schedule\":1263798000000,\"task\":\"move\",\"destination\":{\"predefined_pose\":\"home\"}}],\"initializer\":{\"device_type\":\"ui_loc\",\"device_id\":\"ui_loc_0001\"}}"

tt = json_parser.Tasks(json_parameters)

tt.tas
