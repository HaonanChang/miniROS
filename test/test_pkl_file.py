"""
Test the pickle file.
"""

import pickle


if __name__ == "__main__":
    act_file = "/home/we/Projects/miniROS/tmp_data/20251011_212124_688365/robot_act.pkl"
    obs_file = "/home/we/Projects/miniROS/tmp_data/20251011_212124_688365/robot_obs.pkl"
    with open(act_file, "rb") as f:
        act = pickle.load(f)
    with open(obs_file, "rb") as f:
        obs = pickle.load(f)
    print(act)
    print(obs)