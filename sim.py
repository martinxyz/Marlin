#!/usr/bin/env pylab
from pylab import *
clf()


vmax = 200 # mm/s
amax = 18000 # mm/s2
#amax = 30000 # mm/s2
#jmax = X20.00 # mm/s ????
jmax = 2500000 # mm/s3
xmax = 50 # mm end-position

steps_per_mm = 78.74 # x/y axes
dt = 0.9e-6 # simulation resolution (continous simulation, not stepper)

# for stepper simulation
irq_interval = 50e-6
subpos_microstep = 1<<13

seterr('raise') # exceptions on overflow, please

x = 0.0
v = 0.0
a = 0.0
j = jmax

t = 0.0
stop = 0

data = []

phase = 0
segments = []
segments.append([t, x, v, a, j])


phase_old = None
while True:
    if phase_old != phase:
        print 'phase', phase, 'jerk is', j
        phase_old = phase
    x_step = round(x * steps_per_mm)/steps_per_mm
    data.append((t, x, v, a, x_step))
    t += dt
    a += j*dt
    v += a*dt
    x += v*dt#+0.5*a*dt*dt
    assert x >= 0
    assert v >= -0.0001
    #print phase, x, v, a
    if phase == 0 and a > amax: # end of jerk limit
        phase = 1
        a = amax
        j = 0.0
        segments.append([t, x, v, a, j])
        t_max_a = t
        print 'max accel at %.3f ms' % (t*1000)
        stop = 1
    elif phase == 1 and v > vmax - segments[1][2]: # end of constant acceleration
        t_max_a_ends = t
        phase = 2
        j = -jmax
        segments.append([t, x, v, a, j])
    elif phase == 2 and a <= 0.0: # end of jerk limit
        phase = 3
        j = 0.0
        a = 0.0
        v = vmax
        segments.append([t, x, v, a, j])
        t_max_v = t
        print 'max velocity at %.3f ms' % (t*1000)
    elif phase == 3 and x > xmax - segments[3][1]: # end of constant velocity
        phase = 4
        j = -jmax
        segments.append([t, x, v, a, j])
    elif phase == 4 and a <= -amax: # end of jerk limit
        phase = 5
        j = 0.0
        a = -amax
        segments.append([t, x, v, a, j])
    elif phase == 5 and v <= segments[1][2]: # end of constant deceleration
        phase = 6
        j = +jmax
        segments.append([t, x, v, a, j])
    elif phase == 6 and a <= 0:
        j = 0.0
        a = 0.0
        v = 0.0
        segments.append([t, x, v, a, j])
        break

data = array(data, dtype='float')

def stepper_sim(segments):
    stepper_segments = []
    last_time = None

    accel_scale    = 1<<9
    #velocity_scale = 1<<1
    velocity_scale = 1

    for idx, (t, x, v, a, j) in enumerate(segments):
        if idx == len(segments)-1:
            duration = 0
        else:
            t1 = segments[idx+1][0] 
            duration = int(round((t1 - t)/irq_interval))
        # j is in mm/s**3, convert to steps/(irq_interval**3)
        #print 'j=', j
        j = j * steps_per_mm * subpos_microstep * (irq_interval**3)
        # scale jerk
        j = j * accel_scale * velocity_scale
        #print 'j_new=', j
        j = int(round(j))
        stepper_segments.append((duration, j))
    #print 'stepper_segments', stepper_segments
    assert stepper_segments[0][1] != 0

    segments_testcode = [(10000, 0)]
    segments_testcode += stepper_segments[:] 
    segments_testcode += [(0, 0), (int(0.8/irq_interval), 0)]
    for duration, j in stepper_segments:
        segments_testcode.append((duration, -j))
    segments_testcode += [(0, 0)]
    for idx, (duration, j) in enumerate(segments_testcode):
        print 'motion_buffer[%d] = {.duration=%d, .jerk=%d};' % (idx, duration, j)

    result = []
    axes = 1
    current_subposition = zeros(axes, dtype='int64')
    current_velocity = zeros(axes, dtype='int32')
    current_acceleration = zeros(axes, dtype='int32')
    current_jerk = zeros(axes, dtype='int32')

    #current_subposition = zeros(axes, dtype='float')
    #current_velocity = zeros(axes, dtype='float')
    #current_acceleration = zeros(axes, dtype='float')
    #current_jerk = zeros(axes, dtype='float')

    irq = 0
    v_range = []
    a_range = []
    j_range = []
    for steps, jerk in stepper_segments:
        current_jerk[0] = jerk
        #current_velocity[0]     += (jerk+3)/6 / accel_scale
        current_velocity[0]     += (jerk*5+17)/32 / accel_scale # jerk/6 (approximated)
        current_acceleration[0] += jerk/2
        for step in xrange(steps):
            result.append((irq*irq_interval, round(float(current_subposition[0])/subpos_microstep)/steps_per_mm, float(current_velocity[0])/velocity_scale/subpos_microstep/steps_per_mm/irq_interval))
            irq += 1
            for i in range(axes):
                # Formula: position     += velocity + acceleration/2 + jerk/6
                #          velocity     += acceleration + jerk/2
                #          acceleration += jerk
                #
                # For constant jerk:
                #          initial_velocity     += jerk/6
                #          initial_acceleration += jerk/2
                # New Formula: position     += velocity + acceleration/2
                #              velocity     += acceleration
                #              acceleration += jerk

                #current_subposition[i]  += current_velocity[i] + current_acceleration[i]/2/256 + current_jerk[i]/6/256;
                #current_velocity[i]     += current_acceleration[i]/256 + current_jerk[i]/2/256;
                #current_acceleration[i] += current_jerk[i];

                #current_subposition[i]  += current_velocity[i] + current_acceleration[i]/(accel_scale*2) + current_jerk[i]/(accel_scale*6);
                #current_velocity[i]     += current_acceleration[i]/accel_scale + current_jerk[i]/(accel_scale*2);
                #current_acceleration[i] += current_jerk[i];

                current_subposition[i]  += (current_velocity[i]+velocity_scale/2)/velocity_scale; # + (current_acceleration[i]+accel_scale/4*velocity_scale)/(accel_scale/2*velocity_scale);
                current_velocity[i]     += (current_acceleration[i]+accel_scale/2)/accel_scale;
                current_acceleration[i] += current_jerk[i];

                v_range.append(current_velocity[i])
                a_range.append(current_acceleration[i])
                j_range.append(current_jerk[i])

                #current_subposition[i] += 3*current_velocity[i]/2 + 3*current_acceleration[i]*256 + current_jerk[i]*256;
                #current_velocity[i] += 6*current_acceleration[i]*256 + 3*current_jerk[i]/2;
                #current_acceleration[i] += current_jerk[i];

                #if irq % 100 == 0:
                #    print '...'
                if irq % 1000 < 3:
                    print 'jerk %f accel %f velo %f subpos %f' % (current_jerk[i], current_acceleration[i], current_velocity[i], current_subposition[i])
                #print 'jerk %d accel %d velo %d subpos %d' % (current_jerk[i], current_acceleration[i], current_velocity[i], current_subposition[i])
    
    def rangeprint(s, v):
        lim = max([abs(max(v)), abs(min(v))])
        if lim == 0:
            lim2 = 0
        else:
            lim2 = log2(lim)
        print s+'range: max +/- %d (log2 = %.2f)' % (lim, lim2)
    rangeprint('v_', v_range)
    rangeprint('a_', a_range)
    rangeprint('j_', j_range)
    return array(result)

print 'Starting stepper_sim...'
data_stepper = stepper_sim(segments)
print 'done.'

t=data[:,0]
plot(t, data[:,1]*100, label='position*100')
plot(t, data[:,2], label='velocity')
#plot(t, data[:,3]/100, label='acceleration/100')
plot(t, data[:,4]*100, label='position*100 (microstep)', color='k')
if t_max_a:
    axvline(x=t_max_a, color='k')
if t_max_a_ends:
    axvline(x=t_max_a_ends, color='k')
if t_max_v:
    axvline(x=t_max_v, color='k', linewidth=2)

t = data_stepper[:,0]
plot(t, data_stepper[:,1]*100, label='stepper_sim position*100')
plot(t, data_stepper[:,2], label='stepper_sim velocity')
print 'endpos error: %.6fmm' % abs(data[-1,1] - data_stepper[-1,1])

grid()
legend(loc='upper left')
xlabel('seconds')
show()
