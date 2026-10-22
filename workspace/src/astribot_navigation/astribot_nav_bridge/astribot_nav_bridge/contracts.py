"""Pure validation, intentionally independent of Astribot SDK and ROS imports."""
import math

MAX_LINEAR_SPEED = .1
MAX_ANGULAR_SPEED = .1


def checked_command(values, max_v=MAX_LINEAR_SPEED, max_w=MAX_ANGULAR_SPEED):
    if len(values)!=6 or not all(math.isfinite(float(x)) for x in values):
        raise ValueError('Twist must contain six finite numbers')
    if any(abs(values[i])>1e-9 for i in (1,2,3,4)):
        raise ValueError('Only forward motion and yaw are enabled')
    # Scale both components together: independent clipping changes curvature
    # and makes a path follower take wider turns than its collision prediction.
    v=max(0.,float(values[0])); w=float(values[5])
    scale=max(1.,v/max_v,abs(w)/max_w)
    return v/scale,w/scale

def inspect_state(msg):
    p=list(msg.position); v=list(msg.velocity)
    if len(p)!=3 or len(v)!=3 or not all(math.isfinite(x) for x in p+v):
        raise ValueError('Expected three finite position and velocity values')
    return {'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},
            'frame_id':msg.header.frame_id,'name':list(msg.name),'position':p,'velocity':v,
            'semantics':'unconfirmed','usable_as_odometry':False}
