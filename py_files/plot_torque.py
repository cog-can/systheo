import matplotlib.pyplot as plt

torques = [-20, -10, -8, 5, 8, 10, 20]
derivs = [-9.02, -3.34, -2.08, 3.23, 5.06, 6.23, 11.12]

def get_slope(torques, derivs):
    slope = (derivs[-1] - derivs[0]) / (torques[-1] - torques[0])
    return slope

# slope_1 = (derivs[2] - derivs[0]) / (torques[2] - torques[0])
# slope_2 = (derivs[6] - derivs[3]) / (torques[6] - torques[3])

# print(f"First slope :: {slope_1}")
# print(f"Second slope :: {slope_2}")


plt.plot(torques, derivs, marker='o', label='Torque vs Derivative')

for x, y in zip(torques, derivs):
    plt.text(x, y, f'({x}, {y:.2f})', fontsize=8, ha='right', va='bottom')

plt.xlabel('Torque')
plt.ylabel('Derivative')
plt.title('Torque vs Derivative Relationship')

plt.legend()

plt.show()
