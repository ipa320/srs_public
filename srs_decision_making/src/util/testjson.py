import json_parser

json_parameters = "{\"tasks\":[{\"time_schedule\":1263798000000,\"task\":\"move\",\"destination\":{\"predefined_pose\":\"home\"}}],\"initializer\":{\"device_type\":\"ui_loc\",\"device_id\":\"ui_loc_0001\"}}"

tasks = json_parser.Tasks(json_parameters)

task_feedback = json_parser.Task_Feedback ('11' , tasks.device_id, tasks.device_type, tasks.tasks_list[0].task_json_string)

print task_feedback.task_name

