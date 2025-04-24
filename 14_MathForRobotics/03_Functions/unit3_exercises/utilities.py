import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Polygon
import matplotlib.patches as patches
import numpy as np

def plot_vectors(vectors,meth):
    fig, ax = plt.subplots(figsize=(8, 6))
    # Set the axes through the origin
    for spine in ['left', 'bottom']:
        ax.spines[spine].set_position('zero')
    for spine in ['right', 'top']:
        ax.spines[spine].set_color('none')
        
    if meth=='vectors':
        ax.set(xlim=(-5, 5), ylim=(-5, 5))
    elif meth=='sum':
        ax.set(xlim=(0, 5), ylim=(0, 5))
    elif meth=='subtract':
        ax.set(xlim=(-2, 5), ylim=(0, 5))
    elif meth=='rotation':
        ax.set(xlim=(0, 2), ylim=(0, 2))
        style = "Fancy, tail_width=3, head_width=20, head_length=20"
        kw = dict(arrowstyle=style, color="r")
        a3 = patches.FancyArrowPatch(vectors[0], vectors[1],
                connectionstyle="arc3,rad=.25", **kw)
        plt.gca().add_patch(a3)
        
    ax.grid()

    for v in vectors:
        ax.annotate('', xy=v, xytext=(0, 0),
                    arrowprops=dict(facecolor='blue',
                    shrink=0,
                    alpha=0.7,
                    width=0.5))
        ax.text(1.3 * v[0], 1.3 * v[1], str(v))
    if meth=='sum':
        ax.annotate('', xy=vectors[0]+vectors[1], xytext=vectors[1],
                    arrowprops=dict(arrowstyle='-', 
                    linestyle='--',
                    facecolor='black',
                    alpha=0.7,
                    linewidth=0.9))
    elif meth=='subtract':
        ax.annotate('', xy=vectors[2]+vectors[1], xytext=vectors[1],
                    arrowprops=dict(arrowstyle='-', 
                    linestyle='--',
                    facecolor='black',
                    alpha=0.7,
                    linewidth=0.9))
    plt.show()

def plot_triangle(vectors):
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set(xlim=(0, 5), ylim=(0, 5))
    ax.grid()
    
    for i,v in enumerate(vectors):
        if i==0:
            ax.arrow(0,0,*v,fc='blue',head_width=0.2,alpha=0.9,lw=0.9)
            ax.text(.5 * v[0], .5 * v[1]+0.3, 'x='+str(v),fontsize=12)
        elif i==1:
            #ax.arrow(*v_ant,*v,fc='blue',head_width=0.2,alpha=0.9,lw=0.9)
            ax.text(1.0 * v_ant[0]+0.2, .5 * v[1], 'y='+str(v),fontsize=12)
        elif i==2:
            ax.arrow(0,0,*v,fc='blue',head_width=0.2,alpha=0.9,lw=0.9)
            ax.text(.5 * v[0], .5 * v[1]+1.0, 'hypotenusa='+str(v),fontsize=12)
        v_ant = v
        
    plt.show()

def plot_vectors_3D(vectors):
    X = [0]*len(vectors)
    Y,Z = X,X
    U, V, W = zip(*vectors)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(X, Y, Z, U, V, W)
    ax.set_xlim([-5,5])
    ax.set_ylim([-5,5])
    ax.set_zlim([-5,5])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    
# plot the function
def plot_function(x,f,name):
    fig,ax = plt.subplots()
    ax.spines['left'].set_position('zero')
    #ax.spines['bottom'].set_position('zero')
    plt.xlabel('Values of x')
    plt.ylabel('Function f')
    ax.plot(x,f)
    ax.grid()
    plt.legend([name],loc='upper right')

    plt.show()
    
# plot the points
def plot_points(x,f,name):
    fig,ax = plt.subplots()
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    plt.xlabel('Values of x')
    plt.ylabel('Function f')
    ax.plot(x,f,'bo')
    ax.grid()
    plt.legend([name],loc='upper right')

    plt.show()

# plot the equation
def plot_equation(x,f,name,xlabel,ylabel):
    fig,ax = plt.subplots()
    ax.spines['left'].set_position('zero')
    #ax.spines['bottom'].set_position('zero')
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    ax.plot(x,f)
    ax.grid()
    plt.legend([name],loc='upper right')

    plt.show()


def plot_function_3d(x1,x2,f,name):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(x1,x2,f,label=name)
    surf._edgecolors2d=surf._edgecolors3d
    surf._facecolors2d=surf._facecolors3d
    plt.title('3D function')
    plt.xlabel('x_1')
    plt.ylabel('x_2')
    plt.legend()
    plt.show()
    
def plot_gradient(x_1,x_2,df,name):
    plt.figure()
    plt.xlabel('Values of x_1')
    plt.ylabel('Values of x_2')
    plt.title('Gradient')
    plt.quiver(x_1,x_2,df[0],df[1])
    plt.legend([name],loc='upper right')
    plt.show()
    
def plot_derivative(x,f,d,name,name_d):
    fig,ax = plt.subplots()
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    plt.xlabel('Values of x')
    plt.ylabel('Function f')
    ax.plot(x,f,label=name)
    ax.plot(x,d,label=name_d)
    ax.grid()
    plt.legend(loc='upper right')
    plt.show()
    
def plot_tangent(x,f,t1,t2,name,name_t1,name_t2):
    fig,ax = plt.subplots()
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    plt.xlabel('Values of x')
    plt.ylabel('Function f')
    ax.plot(x,f,label=name)
    ax.plot(x,t1,label=name_t1)
    ax.plot(x,t2,label=name_t2)
    ax.plot(x[2],f[2],'o',linewidth=10)
    ax.plot(x[9],f[9],'o',linewidth=10)
    ax.grid()
    plt.legend(loc='upper right')
    plt.show()
    
def plot_integral(x,y,func,a,b):
    fig, ax = plt.subplots()
    ax.plot(x, y, 'r', linewidth=2)
    ax.set_ylim(bottom=0)

    # Make the shaded region
    ix = np.linspace(a, b)
    iy = func(ix)
    ca = [[ix[i],iy[i]] for i in range(len(ix))]
    verts = [[a, 0]]+ ca + [[b, 0]]
    poly = Polygon(verts, facecolor='0.9', edgecolor='0.5')
    ax.add_patch(poly)

    ax.text(0.5 * (a + b), 30, r"$\int_a^b f(x)\mathrm{d}x$",
            horizontalalignment='center', fontsize=20)

    fig.text(0.9, 0.05, '$x$')
    fig.text(0.1, 0.9, '$y$')

    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.xaxis.set_ticks_position('bottom')

    ax.set_xticks((a, b))
    ax.set_xticklabels(('$a$', '$b$'))
    ax.set_yticks([])

    plt.show()
    
def plot_distro(x,y,meth):
    if meth=='uniform':
        zero = [0.0]
        y = zero+y+zero
    elif meth=='cumulative':
        y[-1] = 0.0

    fig,ax = plt.subplots(1,1,figsize=(4,4))
    ax.plot(x,y)
    ax.fill(x,y,facecolor='lightblue')
    if meth=='uniform' or meth=='cumulative':
        ax.set_xlim([0,8])
        ax.set_ylim([0,0.6])
    elif meth=='normal':
        ax.set_xlim([-2.0,12.0])
        ax.set_ylim([0,1.0])
    plt.xlabel('Values of X')
    plt.ylabel('Probability distribution p')
    plt.show()
