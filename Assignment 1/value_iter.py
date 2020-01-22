import numpy as np

#This function initialises all the variables

def initialisation():
	#This initialises the new states for both course of actions i.e 'save' or 'adverise'
	new_state_a = np.array([0, 0, 0, 0])
	new_state_s = np.array([0, 0, 0, 0])

	#This initialises the state transition probability matrices for the given state transition diagram
	pss_a = np.array([[0.5, 0.5, 0, 0],[0, 1, 0, 0],[0.5, 0.5, 0, 0],[0, 1, 0, 0]])
	pss_s = np.array([[1, 0, 0, 0],[0.5, 0, 0, 0.5],[0.5, 0, 0.5, 0],[0, 0, 0.5, 0.5]])

	#This initialises the new state which is the max over all actions
	new_state = np.array([0, 0, 0, 0])
	curr_state = np.array([0, 0, 0, 0])

	return new_state_s, pss_s, new_state_a, pss_a, new_state, curr_state

#This function does the value itereation which is used to update the new states for each action
def do_value_iter(Rewards, gamma, pss, curr_state):
	return (Rewards + gamma*(pss@curr_state))

#This function evaluates the optimal policy i.e whether to advertise or to save
def get_P(new_state_a, new_state_s, P):
	for i in range(0,4):
		if(new_state_a[i] > new_state_s[i]):
			P[i] = 'advertise'
		if (new_state_a[i] < new_state_s[i]):
			P[i] = 'save'
	return P

if __name__ == "__main__":	

	gamma = 0.9
	Rewards = np.array([0, 0, 10, 10]) #rewards for each state 
	P = ['save', 'save', 'save', 'save'] # initailsatition of policy
	
	new_state_s, pss_s, new_state_a, pss_a, new_state, curr_state  = initialisation() #All variables are initialised

	while(True): #Run the loop until convergence

		curr_state = new_state

		#update the new states for each of the actions
		new_state_a = do_value_iter(Rewards, gamma, pss_a, curr_state)
		new_state_s = do_value_iter(Rewards, gamma, pss_s, curr_state)
		
		#update the new state yielding max reward
		new_state = np.maximum(new_state_a, new_state_s)
		P = get_P(new_state_a, new_state_s, P)
		
		print('The current values for the states are', curr_state)
		print('The current Policy is', P)
		
		#break when convergence occurs
		if(np.array_equal(curr_state, new_state)):
			break

	print('The optimal Policy is', P)
	print('The converged Values for each of the states are', curr_state)