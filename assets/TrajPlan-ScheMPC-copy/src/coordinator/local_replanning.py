from pkg_sche.sp_comsat.Compo_slim import Compo_slim

def repair_robot_path(
    replan_rid,
    handoff_node,
    blocked_edge,
    current_time,
    jobs_list,
    remaining_jobs
):
    # 1. Keep selected tasks and task order.
    # 2. Remove completed part of replanned route.
    # 3. Set current_node as its new starting point.
    # 4. Freeze other robot paths.
    # 5. Change only replanned robot's physical path.
    # 6. Schedule replanned robot around frozen reservations.
    # 7. Return its replacement schedule.

    # print(f'[path_repair] Replanning the path for {replan_rid}')
    # print(f'[path_repair] Replanning robot pose {robot_pose}')
    # print(f'[path_repair] Neighbor nodes {prev_node}, {next_node}')
    # print(f'[path_repair] Graph {graph}')
    # print(f'[path_repair] Current routes: {active_routes}')
    # print(f'[path_repair] Job List: {jobs_list}')
    print(f'[path_repair] Time: {current_time}')
    # print(f'[path_repair] Conflicting node: {current_node}')
    # print(f'[path_repair] Frozen Schedule of other robots: {frozen_schedules}')

    problem = problem_builder(replan_rid, handoff_node, blocked_edge, jobs_list, remaining_jobs)
    # print(f'[path_repair] {jobs_list}')
    instance, optimum, running_time, len_previous_routes, paths_changed, new_solution, new_routes, new_jobs_list\
        = Compo_slim(problem, replanning_robot=replan_rid, current_time=current_time)
    # print(f'[path_repair] New calculated route: {new_routes}')
    # print(f'[path_repair] New job list: {new_jobs_list}')
    # print(f'[path_repair] New calculated route: {new_solution}')

    return {"solution": new_solution,
            "routes": new_routes,
            "jobs_list": new_jobs_list}


def problem_builder(rid, handoff_node, blocked_edge, jobs_list, remaining_jobs):
    """ 
    builds problem for the optimzer to solve
    Returns:
    1. remaining tasks of the robot being replanned
    2. frozen task list, schedule of the other robots 
    """
    # removing precedence from first job
    jobs_list[rid][remaining_jobs[rid][0]]["precedence"] = []

    jobs = {job_id: job_data.copy() 
                for job_id, job_data in jobs_list[rid].items()
                if job_id in remaining_jobs[rid]
            }
    jobs[remaining_jobs[rid][0]]["precedence"] = []
    # print(f'\n\n\n\n [path_repair] jobs: {jobs}')

    # TODO: hardcoded intermediate node as N99, needs to change for ideal scenario
    problem = {
        "test_data": {
            "Big_number": 500,
            "Autonomy": 500,
            "charging_coefficient": 1,
            "Environment": "CoordScene2", 
            "nodes": {
                "N00":{"x":1,"y":1,"next":["N01", "N10"]},
                "N01":{"x":1,"y":3,"next":["N00", "N11", "N02"]},
                "N02":{"x":1,"y":5,"next":["N01", "N12"]},
                "N10":{"x":3,"y":1,"next":["N00", "N20", "N11"]},
                "N11":{"x":3,"y":3,"next":["N10", "N21", "N12", "N01"]},
                "N12":{"x":3,"y":5,"next":["N02", "N11", "N22"]},
                "N20":{"x":5,"y":1,"next":["N10", "N21"]},
                "N21":{"x":5,"y":3,"next":["N20", "N11", "N22"]},
                "N22":{"x":5,"y":5,"next":["N21", "N12"]},
                # "N99":{"x":pose[0],"y":pose[1],"next":[next_node, prev_node]},
            },
            "hub_nodes": []
        },
        "ATRs": {
            rid: handoff_node
        },
        "jobs": jobs
    }

    blocked_from, blocked_to = blocked_edge
    problem["test_data"]["nodes"][blocked_from]["next"].remove(blocked_to)

    # problem["test_data"]["nodes"][prev_node]["next"].append("N99")
    # problem["test_data"]["nodes"][next_node]["next"].append("N99")

    # print(problem["test_data"]["nodes"][prev_node]["next"])
    # print(problem["test_data"]["nodes"][next_node]["next"])
    # print(problem["test_data"]["nodes"]["N99"]["next"])
            
    return problem