"""Python2/3-compatible one-shot gait policy; no ROS or physical I/O."""
COMMANDS={'w':(0.05,0.,0.),'s':(-0.04,0.,0.),'a':(0.,0.03,0.),'d':(0.,-0.03,0.),'q':(0.,0.,10.),'e':(0.,0.,-10.)}

class KeyEdges(object):
    def __init__(self):
        self.released=False
    def poll(self,keys,focused):
        if not focused:
            self.released=False
            return None
        if not keys:
            self.released=True
            return None
        if not self.released:
            return None
        self.released=False
        return keys[0] if len(keys)==1 and keys[0] in COMMANDS else None

class Policy(object):
    def __init__(self,ttl=.8,pending_timeout=3.,execution_timeout=10.):
        self.ttl=ttl
        self.pending_timeout=pending_timeout
        self.execution_timeout=execution_timeout
        self.armed=False
        self.phase='disarmed'
        self.hb_seq=-1
        self.event_id=-1
        self.last_hb=None
        self.healthy=False
        self.key=None
        self.submitted=None
        self.dispatched=None
        self.saw_busy=False
    def heartbeat(self,seq,now,healthy):
        if seq<=self.hb_seq: return False
        self.hb_seq=seq
        self.last_hb=now
        self.healthy=bool(healthy)
        return True
    def fresh(self,now):
        return self.healthy and self.last_hb is not None and 0<=now-self.last_hb<=self.ttl
    def arm(self,now):
        if self.phase not in ('disarmed','idle') or not self.fresh(now): return False
        self.armed=True
        self.phase='idle'
        return True
    def submit(self,key,event_id,now):
        if event_id<=self.event_id: return 'duplicate'
        self.event_id=event_id
        if not self.armed: return 'not_armed'
        if self.phase!='idle': return 'busy_not_queued'
        if not self.fresh(now): return 'stale_link'
        if key not in COMMANDS: return 'unknown_key'
        self.key=key
        self.submitted=now
        self.phase='pending'
        return 'accepted'
    def take(self,now):
        if self.phase!='pending' or not self.armed or not self.fresh(now): return None
        if not 0<=now-self.submitted<=self.pending_timeout: return None
        self.phase='executing'
        self.dispatched=now
        self.saw_busy=False
        return COMMANDS[self.key]
    def observe(self,value,now):
        if self.phase!='executing' or now<self.dispatched: return False
        if value>0:
            self.saw_busy=True
        elif value==0 and self.saw_busy:
            self.phase='idle'
            self.key=None
            return True
        return False
    def fault(self,now):
        if not self.armed: return None
        if not self.fresh(now): return 'heartbeat_or_camera_expired'
        if self.phase=='pending' and not 0<=now-self.submitted<=self.pending_timeout: return 'no_gait_request'
        if self.phase=='executing' and not 0<=now-self.dispatched<=self.execution_timeout: return 'completion_feedback_timeout'
        return None
    def cancel(self):
        self.armed=False
        self.phase='disarmed'
        self.key=None

