import numpy as np
import matplotlib.pyplot as plt
import numpy.testing as npt
import random
import matplotlib.patches as patches
import matplotlib.animation as animation
from IPython.display import HTML
k =1.38e-27

def resolve_sphere_collision(state,id0, id1):
    '''
    Given two spheres p0 and p1 which
    are currently colliding (exactly in contact), returns an updated state
    where the sphere velocities have been updated to account for their collision.
    '''
    #
    p0 =state[id0]
    p1 =state[id1]
    m0,r0,x0,y0,vx0,vy0 = p0[0],p0[1],p0[2],p0[3],p0[4],p0[5]
    m1,r1,x1,y1,vx1,vy1 = p1[0],p1[1],p1[2],p1[3],p1[4],p1[5]
    state.remove(state[id0])
    state.remove(state[id1])

    # define impact vector
    bx = (x1-x0)/(r0+r1)
    by = (y1-y0)/(r0+r1)

    c = (-by,bx)
    b = ( bx,by)
    # rotated system
    # v1 = v1 B +v1c c
    # v2 =v2b b + v2c c
    v1x =  vx0*bx + vx0*bx
    v1y =  vy0*by + vy0*by
    v2x =  vx1*bx +vx1*bx
    v2y =  vy1*by + vy1*by

    v1x_new = vx1*bx + vx0*bx
    v1y_new = vy1*by - vy0*by

    v2x_new = vx0*bx + vx1*bx
    v2y_new = vy0*by - vy1*by

    state.insert(id0,my_sphere(m0,r0,x0,y0,v1x_new,v1y_new))
    state.insert(id1,my_sphere(m1,r1,x1,y1,v2x_new,v2y_new))
    #print('states, appended ')


    return state

def resolve_wall_collision(cur_state, id0, wall):
    '''
    Given the pre-collision
    state, a sphere index id0, and a wall label (one of N/S/E/W),
    returns an updated state where the velocity of the indicated
    sphere has been updated to account for its collision with the wall.
    '''
    particle = cur_state[id0]
    m,r,x,y,vx,vy = particle[0],particle[1],particle[2],paricle[3],particle[4],particle[5]
    cur_state.remove(cur_state[id0])
    if wall == 'N':
        #flip vy
        vy=-vy
    if wall == 'S':
        #flip vy
        vy=-vy
    if wall == 'E':
        #flip vx
        vx=-vx
    if wall == 'W':
        #flip vx
        vx=-vx
    cur_state.insert(id0,my_sphere(m,r,x,y,vx,vy))

    # if problems with this function in implemantation remive particle from current state
    # append new particle with updated positions after removing

    return cur_state

def find_first_collision(state, sphere_hits, wall_hits, L):
        '''
        Given a
        (pre-collision!) state, two lists of expected collision tuples
        sphere_hits and wall_hits as returned by find_overlaps_in_state and
        find_wall_collisions_in_state, and the box size L, find and
        return (collision_type, collision, dt) corresponding
        to the first collision that happens (smallest time to collision.)
        '''

        # spere hits =  (indie 1, indice 2)
        # wall hits =( indice , wall)
        # returns (collision type , (collision indices), elapsed time)
        t_list=[]
        for hit in sphere_hits:
            t_list.append(("sphere", hit, compute_time_to_sphere_collision(state,hit[0],hit[1])))
        for hit in wall_hits:
            t_list.append(('wall', hit,compute_time_to_wall_collision(state,hit[0],L,hit[1])))

        #sort
        times = [it[2] for it in t_list]
        indi = times.index(min(times))

        return t_list[indi]

def compute_time_to_wall_collision(state, id0, L, wall):
    '''
    Given a sphere
    index id0 (labeling its index within state), the box size L, and a wall
    label (one of N/S/E/W), compute and return the time elapsed dt
    from the sphere's position in state until it collides
    with the given wall.
    '''
    particle = state[id0]
    if wall == 'N':
        dy = L - state[id0][3]
        vy = state[id0][5]
        if vy>0:
            t = np.abs(dy/vy)

    if wall == 'S':
        dy = state[id0][3]
        vy = state[id0][5]
        if vy<0:
            t = np.abs(dy/vy)

    if wall == 'E':
        dx = L - state[id0][2]
        vx = state[id0][4]
        if vx<0:
            t = np.abs(dx/vx)

    if wall == 'W':
        dx = L - state[id0][2]
        vx = state[id0][4]
        if vx>0:
            t = np.abs(dx/vx)


    return t

def compute_time_to_sphere_collision(state, id0, id1):
    '''
    Given two sphere indices id0 and id1
    (labeling their index within state), compute and return
    the time elapsed dt from their positions in state until
    they collide.
    '''
    # assuming spheres will collide
    # find point of collision fom vector use simple 2d kinemtics to find t

    p1 =state[id0]
    p2 = state[id1]

    r0= p2[1]
    r1 = p1[1]

    x1,y1 = p1[2],p1[3]
    x2,y2 = p2[2],p2[3]
    vx1,vy1 = p1[4],p1[5]
    vx2,vy2 = p2[4],p2[5]


        # solving kinematic equations
    dx = (x2-x1)
    dy = (y2-y1)
    dvx =(vx2 - vx1)
    dvy = (vy2- vy1)

    R2  = (r0+r1)**2

    dx2 =dx**2 + dy**2
    dv2 = dvx**2 + dvy**2


    dx_dot_dv = dx*dvx + dy*dvy

    t0 = 1/dv2 *( (-dx_dot_dv)+ np.sqrt((dx_dot_dv)**2 - dv2*(dx2 - R2)))
    t1 = 1/dv2 *( (-dx_dot_dv)- np.sqrt((dx_dot_dv)**2 - dv2*(dx2 - R2)))

    if t0>t1 and t1>0:
        return t1
    else :
        return t0
def find_wall_collisions_in_state(state, L):
    '''
    Given a state, returns a list of
    pairs of spheres and walls which they are overlapping (or past), where
    each pair is a tuple containing the list index of a sphere and
    a cardinal direction (NSEW) indicating which wall it is overlapping/past.
    '''
    # use same method as find overlap but compare to fixed pos
    x=[]
    y=[]


    for m0, r0, x0, y0, vx0, vy0 in state:
        x.append(x0)
        y.append(y0)

        r=r0
    #x>(x[0]+.01)and x<(x[0]-.01)
    x = np.array(x)
    y = np.array(y)

    #define wall positions
    N,S,E,W = (L,0,0,L)

    #check if x or y positions fall within r of walls
    N_hits = y>= (L-r)
    S_hits = y<= (r)
    E_hits = x<= (r)
    W_hits = x>= (L-r)



    #iterate through truth table and catalog hits/direction
    wall_hits=[]
    for i in range(len(state)):
        if N_hits[i]==True :
            wall_hits.append((i,'N'))
        if S_hits[i]==True:
            wall_hits.append((i,'S'))
        if E_hits[i]==True:
            wall_hits.append((i,'E') )
        if W_hits[i]==True:
            wall_hits.append((i,'W') )

    return wall_hits
## did not completley test functions above
## output may not be what is expected
def compute_safe_timestep(state):
    ''' Given a state, return a timestep dt which is
    'small enough' that our simulation will be accurate. You may find it use
    ful to find the maximum speed using compute_speeds(state) from above here.
    '''
    # dt < r_min/2vmax
    v=[]
    for m0, r0, x0, y0,vx0,vy0 in state:
        vel = np.sqrt(vx0**2 + vy0**2)
        v.append(vel)
        r=r0
    max_v=max(v)
    dt = r/(2*max_v)-1e-2
    return dt

def my_sphere(m,r,x,y,vx,vy):
    return (m,r,x,y,vx,vy)

def find_overlaps_in_state(state):

    x=[]
    y=[]
    xy=[]
    for m0, r0, x0, y0, vx0, vy0 in state:
        x.append(x0)
        y.append(y0)
        r=r0
    #x>(x[0]+.01)and x<(x[0]-.01)
    for i in range(len(x)):
        #print('dx , dy for particle of index',i)

        # reinitalizes dx and dy at every particle
        dx = [abs(x[it]-x[i]) for it in range(len(x))]
        dy = [abs(y[it]-y[i]) for it in range(len(y))]
        #print(dy,dx)

        # i is particle
        #overlap = [(i,it) for it in range (len(x)) if ]
        #now we have lists of dx / dy
        for j in range(len(x)):

               if dx[j]<(2*r) and dy[j]<(2*r) and i!=j and (j,i) not in xy:
                    #print(i,j)
                    xy.append((i,j))

    return xy

def update_sphere_positions(state,dt):
    # given my_sphere tuple and dt evolves pos into future
    # state is mass ,radius ,x pos, y pos, v x, v y
    # check if dt in proper timestep

    new_state = []
    for it in state:
        new_state.append(my_sphere(it[0],it[1],it[2]+it[4]*dt,it[3]+it[5]*dt,it[4],it[5]))
    return new_state

def plot_state_frame(ax, state, color='blue'):
    # Extract positions
    r = []
    x = []
    y = []
    for m0, r0, x0, y0, vx0, vy0 in state:
        r.append(r0)
        x.append(x0)
        y.append(y0)


    # plt.scatter doesn't work for this -- scatter can't get size in data units, only "points"
    circles = []
    for nn in range(len(state)):
        circles.append(ax.add_artist(patches.Circle(xy=(x[nn], y[nn]), radius=r[nn], facecolor=color)))

    return circles

def plot_state( state, L,color='blue'):
    #fig, ax = plt.subplots(figsize=(8,8))
    #plt.xlim=(0,L)
    #plt.ylim=(0,L)
    # Extract positions
    r = []
    x = []
    y = []
    for m0, r0, x0, y0, vx0, vy0 in state:
        r.append(r0)
        x.append(x0)
        y.append(y0)


    for nn in range(len(state)):
        plt.gca().add_artist(patches.Circle(xy=(x[nn], y[nn]), radius=r[nn], facecolor=color))
        plt.xlim((0,L))
        plt.ylim((0,L))

    return

def random_initial_state(N_particles, L, v, r, m):
    '''
    Given the number of
    particles N_particles, box size L, initial speed v, sphere size r,
    and mass m, returns a state (list of spheres) corresponding to a valid
    initial state where all particles start with the same speed v,
    but random positio)ns and moving in random directions. "Valid" here means:
    no particles can be overlapping each other or the edges of the box.
    '''


    ## check overlap or create x,y random arrays with distance r
    ## could be done O(n^2) but creating grid where no two points are <r and randomly setting them
    x =np.arange(1*r,L-r,2*r)
    y =np.arange(1*r,L-r,2*r)
    # with L/r -1 points

    x , y = np.meshgrid(x,y)
    points=len(x[0])-1

    xy =[(x[random.randint(0,points)][random.randint(0,points)], y[random.randint(0,points)][random.randint(0,points)] )for i in range(N_particles)]

    xy =[it for it in xy if xy.count(it)==1] # gets rid of duplicates

    if len(xy)<N_particles:
        # if overlap not operationaly taing to reset program
        # append x,y components N_p -xy times

        return random_initial_state(N_particles, L, v, r, m)

    rand_direction =np.random.randint(0,2,N_particles)
    rand_direction[rand_direction<1]=-1


    rand_direction1 =np.random.randint(0,2,N_particles)
    rand_direction1[rand_direction1<1]=-1

    vx = rand_direction*v*np.random.random_sample(N_particles)
    vy = rand_direction1*((v**2-vx**2)**.5)


    particles = [my_sphere(m,r,xy[i][0],xy[i][1],vx[i],vy[i]) for i in range(N_particles)]

    return particles

def get_state_at_time(t, times, states):
    '''
    from a ompetes simulation (the output of simulate gas below )
    get the state at time t by interpolating between the known
    states and times . (we dound all collisions, so any interpolation
    is guaranteed free motion
    )
    times = list of times when collisions happens
    states = corresponding list of states for time at index
    '''
    # find state at time in list before
    # iterate through times find time in list right before specified times
    if t==0:
        return states[0]
    for it in range(len(times)):
        if times[it] < t and times[it + 1]>t:
            return update_sphere_positions(states[it],t-times[it])

def run_API_tests():
    return None
    '''
    implement 6 different usefful test functions on for collision run_API_tests
    if all pass return None
    '''

def simulate_gas(initial_state, max_time, L):
    """
    Main loop for hard-sphere kinetic gas project.

    Arguments:
    =====
    * initial_state: List of "sphere" tuples giving the initial state.
        A sphere tuple has the form:
            (m, r, x, y, vx, vy)
        where m is the mass, r is the radius, x and y are the coordinates, and
        vx and vy are the velocity.
    * max_time: The maximum amount of time to allow the system to evolve for.
        (Units depend on how you implement the API - be consistent!)
    * L: Side length of the square (LxL) box in which the simulation is run.

    Returns:
    =====
    (times, states): Two lists containing the times at which any collisions occurred,
        and the state of the system immediately after each collision.

    Example usage:
    =====
    >> state0 = random_initial_state(10, 200, 10, 2, 1)
    >> (times, states) = simulate_gas(state0, 60, 10)

    Now you can run measurements on `states`, or use the plot_state()
    function below to plot states after each collision, or even make an
    animation with get_state_at_time().

    """


    times = [0]
    states = [initial_state]
    eps = 1e-3  # "Overshoot" factor for moving past safe timestep - this should be small!

    time = 0
    cur_state = initial_state

    while time < max_time:

        dt = compute_safe_timestep(cur_state)
        print("Safe timestep = %g" % dt)

        # Try advancing state
        proposed = update_sphere_positions(cur_state, dt)

        # Check for wall or inter-sphere collisions
        sphere_hits = find_overlaps_in_state(proposed)
        wall_hits = find_wall_collisions_in_state(proposed, L)

        print(" %d sphere hits, %d wall hits" % (len(sphere_hits), len(wall_hits)))

        if len(sphere_hits) == 0 and len(wall_hits) == 0:
            # Nothing interesting happened.  Keep state and move on
            time += dt
            cur_state = proposed
            print("Nothing interesting happened at time", time)
            continue

        # Find first collision and what kind it was
        collision_type, first_collision, dt = find_first_collision(cur_state, sphere_hits, wall_hits, L)

        print("%s collision, (%s), dt = %g" % (collision_type, first_collision,  dt))

        # Handle collision
        proposed = update_sphere_positions(cur_state, dt * (1+eps))

        if collision_type == "sphere":
            id0, id1 = first_collision

            # Resolve elastic collision and plug back in to state
            proposed = resolve_sphere_collision(proposed, id0, id1)
            print('sphere collision resolved')


        elif collision_type == "wall":
            id0, wall0 = first_collision

            # Update state to account for wall collision
            proposed = resolve_wall_collision(proposed, id0, wall0)
            print('wall collision resolved')


        # Save new state and time
        time += dt
        cur_state = proposed
        times.append(time)
        states.append(cur_state)
        print("Saving new state at time", time)

    # Save final state
    times.append(time)
    states.append(cur_state)
    return (times, states)

def animate_frames(times,states):
    fig, ax = plt.subplots(figsize=(8,8))

    ax.set_xlim(0,L)
    ax.set_ylim(0,L)



    frames = []
    for t in np.linspace(0,10,50):

        frames.append(plot_state_frame(ax, get_state_at_time(t, times, states)))

    ani = animation.ArtistAnimation(fig, frames, interval=100, blit=True, repeat=False)
    plt.close(fig)  ## Stops Jupyter from showing the last frame alongside the animated plot

    HTML(ani.to_jshtml())
# use declaritive programing to initilize spheres with random gas as
print('this')
