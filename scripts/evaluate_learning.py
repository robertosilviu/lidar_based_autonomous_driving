import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
	steps = []
	td_errors = []
	rewards = []
	
	episodes = 0

	print("opening statistics file")
	with open("../episodes_stats.txt", "r") as f:
		episodes = int(f.readline())
		#print(episodes)
		for line in f:
			data = line.strip().split(" ")
			
			steps.append(int(data[0].replace(',', '.')))
			td_errors.append(float(data[1].replace(',', '.')))
			rewards.append(float(data[2].replace(',', '.')))
		print("ending file reading...")

	f.close()
	
	print("ploting data...")
	ep_list = list(range(1,episodes+1))
	fig, plts = plt.subplots(3, sharex=True)
	fig.suptitle(" Episodes stats")
	plts[0].plot(ep_list, steps, "o", ms=0.5)
	plts[0].set_title("Steps")
	plt.xlabel("Episode")
	plts[0].set_ylabel("Steps")
	#plt.plot(ep_list, steps, 'o:b')
	#plt.show()

	plts[1].plot(ep_list, td_errors, "o", ms=0.5)
	plts[1].set_title(" Temporal difference")
	plts[1].set_ylabel("td_error")
	#plt.show()

	plts[2].plot(ep_list, rewards, "o", ms=0.5)
	plts[2].set_title(" Total reward")
	plts[2].set_ylabel("Reward")
	plt.show()