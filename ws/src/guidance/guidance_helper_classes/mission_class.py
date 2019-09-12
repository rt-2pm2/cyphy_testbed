# Guidance class
import numpy as np
import enum

class TrajectoryType(enum.Enum):
    FullTrj = 0
    AttTrj = 1

class MissionType(enum.Enum):
    Simple = 0
    Composite = 1

################ MISSION CLASS ##################
# This class contains the information about the 
# current mission.
class Mission:
    def __init__(self):
        self.start_pos = np.zeros(3, dtype=float)        
        self.start_vel = np.zeros(3, dtype=float) 
        self.start_acc = np.zeros(3, dtype=float)
        
        self.end_pos = np.zeros(3, dtype=float)        
        self.end_vel = np.zeros(3, dtype=float) 
        self.end_acc = np.zeros(3, dtype=float)

        self.t_start = 0.0
        self.t_stop = np.array([0.0]) 

        # Variables for storing the current control ref
        self.X = np.zeros(3, dtype=float)
        self.Y = np.zeros(3, dtype=float)
        self.Z = np.zeros(3, dtype=float)
        self.W = np.zeros(3, dtype=float)
        self.R = np.eye(3)
        self.Omega = np.zeros(3, dtype=float)

        self.isActive = False 
        self.TrjType = TrajectoryType.FullTrj 
        self.MissType = MissionType.Simple
 
    def update(self, p, tg_p, trj_gen, start_time, stop_time,
            v = None, a = None, tg_v = None, tg_a = None, mtype = MissionType.Simple):
        # Update the starting point
        self.start_pos = p
        if (v is not None):
            self.start_vel = v
        else:
            self.start_vel = np.zeros(3, dtype=float)
        if (a is not None):
            self.start_acc = a
        else:
            self.start_acc = np.zeros(3, dtype=float)

         # Update the target point
        self.end_pos = tg_p
        if (tg_v is not None):
            self.end_vel = tg_v
        else:
            self.end_vel = np.zero(3, dtype=float)
        if (tg_a is not None):
            self.end_acc = tg_a
        else:
            self.end_acc = np.zero(3, dtype=float)
    
        self.trj_gen = trj_gen
         
        self.t_start = start_time
        self.t_stop = stop_time

        self.NumberOfPieces = stop_time.size

        self.TrjType = TrajectoryType.FullTrj
        self.MissType = mtype
        self.isActive = True
    
    def getRef(self, t):
        rel_t = t - self.t_start

        # Check whether we are over the first piece
        if (self.MissType == MissionType.Composite):
            if (t > self.t_stop[0]):
                self.TrjType = TrajectoryType.AttTrj

        (X, Y, Z, W, R, Omega) = self.trj_gen.eval(rel_t)
        X[0] = X[0] + self.start_pos[0]
        Y[0] = Y[0] + self.start_pos[1]
        Z[0] = Z[0] + self.start_pos[2]

        return (X, Y, Z, W, R, Omega)
        

    def queryStatus(self, t):
        """
        Return the status of the Mission
        looking at the time of the planned
        trajectory.
        """
        if (t > np.max(self.t_stop)):
            self.isActive = False
            return self.isActive
        else:
            self.isActive = True 
            return self.isActive
        
    def queryMissionType(self):
        """
        Return the type of mission: Composite or Simple
        """
        return self.MissType

    def queryTrjType(self):
        """
        Return the type of trajectory to track: Full or Att
        """
        return self.TrjType

    def getStartTime(self):
        return self.t_start

    def getStopTime(self):
        return self.t_stop
    
    def getStart(self):
        return self.start_pos

    def getEnd(self):
        return (self.end_pos, self.end_vel, self.end_acc)