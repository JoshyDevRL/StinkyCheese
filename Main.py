from util.objects import *
from util.routines import *
from util.tools import *
from util.custom import *
import random

class Bot(CheeseAgent):
    def run(agent):
        home = agent.friend_goal.location - Vector3(0, side(agent.team)*1000, 100)
        leftfield = Vector3(-side(agent.team)*4200, agent.ball.location.y + (-side(agent.team)*2000), 200)
        rightfield = Vector3(side(agent.team)*4200, agent.ball.location.y + (-side(agent.team)*2000), 200)
        targets = {'foe_goal': (agent.foe_goal.left_post, agent.foe_goal.right_post), 'away_from_our_net': (
            agent.friend_goal.right_post, agent.friend_goal.left_post), 'upfield': (leftfield, rightfield)}
        hits = find_hits(agent, targets)
        closest_large_boost = agent.get_closest_large_boost()
        ball_distance = agent.me.location.dist(agent.ball.location)
        ball_to_goal = agent.friend_goal.location.dist(agent.ball.location)
        div = 1 if agent.team == 1 else -1

        dribbler = agent._dribbler.GetDribbler(agent, agent.delta_time)
        valid_dribbler = True if dribbler != None and dribbler.team != agent.team and agent._dribbler.Duration() > 0.4 else False
        if agent.index == 0 and False:
            agent.text2d("Dribbler Found: " + str(valid_dribbler), Vector3(40, 40, 0), 2, 2)
            agent.text2d("Dribble Duration: " + str(agent._dribbler.Duration()), Vector3(40, 70, 0), 2, 2)
            agent.text2d("Ball-Z-Check: " + str(agent.ball.location.z > agent._dribbler.z_req), Vector3(40, 100, 0), 2, 2)
            agent.text2d("Dist-Check: " + str(agent.foes[0].location.dist(agent.ball.location) < agent._dribbler.dist_req), Vector3(40, 130, 0), 2, 2)
            agent.text2d("Car-Check: " + str(agent.foes[0].index == (agent._dribbler.prev_car.index if agent._dribbler.prev_car is not None else -1)), Vector3(40, 160, 0), 2, 2)
            agent.text2d("Car-Info: " + str(agent.foes[0].index) + " || " + str(agent._dribbler.prev_car.index), Vector3(40, 190, 0), 2, 2)

        nobody_back = True
    
        if agent.intent == None or agent.intent.__class__.__name__ == "goto":
            if agent.kickoff_flag:
                agent.set_intent(SpeedKickoff())
                return

            else:    
                if len(hits['upfield']) > 0 and abs(agent.friend_goal.location.y - agent.ball.location.y) < 6000:
                    agent.set_intent(hits['upfield'][0])
                    return
            
                elif len(hits['foe_goal']) > 0:
                    agent.set_intent(hits['foe_goal'][0])
                    return
            

                elif len(hits['away_from_our_net']) > 0 and abs(agent.friend_goal.location.y - agent.ball.location.y) < 1200:
                    agent.set_intent(hits['away_from_our_net'][0])
                    return
                
                else:
                    if (agent.me.boost < 20 and ball_to_goal > 3200) and closest_large_boost != None:
                        agent.set_intent(goto_boost(closest_large_boost))
                        return
                    
                    else:
                        agent.set_intent(goto(agent.friend_goal.location))
                        return
            

             

