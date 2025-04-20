import sympy as sp

# Define symbolic variables for the DH parameters
th1, th2, th3, th4, th5, th6 = sp.symbols('th1 th2 th3 th4 th5 th6 ')
alf1, alf2, alf3, alf4, alf5, alf6 = sp.symbols('alf1 alf2 alf3 alf4 alf5 alf6')
a1, a2, a3, a4, a5, a6 = sp.symbols('a1 a2 a3 a4 a5 a6 ')
d1, d2, d3, d4, d5, d6 = sp.symbols('d1 d2 d3 d4 d5 d6')

# Define numerical values for the variables for System 1
a1_v1 = 10.25
a2_v1 = 0
a3_v1 = 0
a4_v1 = 0.75
a5_v1 = 0.75
a6_v1 = 1.01
d1_v1 = 0
d2_v1 = 0
d3_v1 = 5.02
d4_v1 = 0.38
d5_v1 = 0
d6_v1 = 0
alf1_v1 = 0
alf2_v1 = -sp.pi/2
alf3_v1 = sp.pi/2
alf4_v1 = 0
alf5_v1 = 0
alf6_v1 = 0

# Assign numerical values to the symbolic variables for System 1
dh_params_vues1 = [
    (a1, a1_v1),
    (a2, a2_v1),
    (a3, a3_v1),
    (a4, a4_v1),  
    (a5, a5_v1),
    (a6, a6_v1),

    (d1, d1_v1),
    (d2, d2_v1),
    (d3, d3_v1),
    (d4, d4_v1),
    (d5, d5_v1),
    (d6, d6_v1),

    (alf1, alf1_v1),
    (alf2, alf2_v1),
    (alf3, alf3_v1),
    (alf4, alf4_v1),
    (alf5, alf5_v1),
    (alf6, alf6_v1),
]

# Define DH parameters for System 1
dh_params1 = [
    (th1, alf1_v1, a1_v1, d1_v1),
    (th2-sp.pi/2, alf2_v1, a2_v1, d2_v1),
    (th3, alf3_v1, a3_v1, d3_v1),
    (th4+sp.pi/2, alf4_v1, a4_v1, d4_v1),
    (th5, alf5_v1, a5_v1, d5_v1),
    (th6, alf6_v1, a6_v1, d6_v1)
]

# Initialize transformation matrices for System 1
T1 = sp.eye(4)

# Function to print validation matrix
def print_validation_matrix(th_v, dh_params_val, system_num):
    # Initialize transformation matrix for validation
    T_val = sp.eye(4)

    # Calculate the transformation matrix for each joint with the substituted values for validation
    for params, angle in zip(dh_params_val, th_v):
        th, alf, a, d = params
        A_val = sp.Matrix([
            [sp.cos(angle), -sp.sin(angle)*sp.cos(alf), sp.sin(angle)*sp.sin(alf), a*sp.cos(angle)],
            [sp.sin(angle), sp.cos(angle)*sp.cos(alf), -sp.cos(angle)*sp.sin(alf), a*sp.sin(angle)],
            [0, sp.sin(alf), sp.cos(alf), d],
            [0, 0, 0, 1]
        ])
        

        T_val = T_val * A_val

    T_val[0,3]+=3

    # Display the validation matrix
    print("Validation Matrix for System {}:".format(system_num))
    sp.pprint(T_val, use_unicode=True)
    print("")

# Calculate the transformation matrix for each joint with the substituted values for System 1
for params in dh_params1:
    th, alf, a, d = params
    A = sp.Matrix([
        [sp.cos(th), -sp.sin(th)*sp.cos(alf), sp.sin(th)*sp.sin(alf), a*sp.cos(th)],
        [sp.sin(th), sp.cos(th)*sp.cos(alf), -sp.cos(th)*sp.sin(alf), a*sp.sin(th)],
        [0, sp.sin(alf), sp.cos(alf), d],
        [0, 0, 0, 1]
    ])
    T1 = T1 * A
    # Add 3 to the last element in the first row of T1 matrix


# Display the final transformation matrix for System 1
print("Transformation Matrix for System 1 (T1):")
sp.pprint(T1, use_unicode=True)
print("")

# Validation Matrices
th_v = [
    [0, -sp.pi/2, 0, sp.pi/2, 0, 0, 0],
    [0, -sp.pi/2, 0, sp.pi/2, 0, sp.pi/2, 0],
    [0, -sp.pi/2, 0, sp.pi/2, sp.pi/2, 0, 0],
    [0, 0, 0, sp.pi/2, 0, 0, 0],
    [sp.pi/2, -sp.pi/2, 0, sp.pi/2, 0, 0, 0],
]

for i, angles in enumerate(th_v):
#     # Define DH parameters for validation
    dh_params_val = [
        (th1, alf1_v1, a1_v1, d1_v1),
        (th2, alf2_v1, a2_v1, d2_v1),
        (th3, alf3_v1, a3_v1, d3_v1),
        (th4, alf4_v1, a4_v1, d4_v1),
        (th5, alf5_v1, a5_v1, d5_v1),
        (th6, alf6_v1, a6_v1, d6_v1)
    ]

    # Use the function to print the validation matrix
    print_validation_matrix(angles, dh_params_val, 1)
