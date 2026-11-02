# AdvantageScope
AdvantageScope is a data displaying tool used to display and visualize data from your Robot. You can use it to display your Real Robot, run Simulations, or even Replay old matches from your Robot.

# Setup
1. Open AdvantageScope
2. Navigate to **App** → **Use Custom Assets Folder**, and open [AdvantageScope](/AdvantageScope/)
3. Navigate to **File** → **Import Layout...**, and open [AdvantageScope Layout.json](/AdvantageScope/AdvantageScope%20Layout.json). The window will close and reopen.
4. In the bottom tab titled **"Poses"**, click the white cursor icon, and select **"Cranberry Alarm"**.
5. At the top of window, click the **"3D Field - For Real Odom Testing"** tab and repeat step 4.
6. Navigate to **App** → **Show Preferences...**, and in Robot Address, enter the correct Robot Address based on your Team Number. (E.g., for "7287", the correct Robot Address is **"10.72.87.2"**.) 

The next steps activate the logging data display, and depends on Real Robot vs Simulation.

### Real Robot
7. Navigate to and select **File** → **Connect to Robot** → **NetworkTables 4 (AdvantageKit)**. (Shortcut is **Ctrl + K** on Windows/Linux or **Command + K** on Mac.)

### Simulation
7. Navigate to and select **File** → **Connect to Simulator** → **NetworkTables 4 (AdvantageKit)**. (Shortcut is **Ctrl + Shift + K** on Windows/Linux or **Shift + Command + K** on Mac.)

## Popping out windows
You can also have a pop-out of a window by entering the tab you want to delete, and select the two rectangles with a "+" in the middle found on the top-right side of the window. 

## Reducing load
The more tabs you have opened, the slower AdvantageScope is. If your computer is struggling, consider closing any tabs you don't need (especially pop-out windows). You can do so by entering the tab you want to delete, and select the Octagon with an "X" in the middle found on the top-right side of the window. You can close pop-out windows like any normal window.

## Common Errors
Here are some common errors and how to resolve them.

1. <u>It connected to the Robot/Simulator, but isn't displaying the right data.</u> <br>
Follow the corresponding Step 7 for Robot/Simulator depending on your target, but select **NetworkTables 4**. If the desired data is back, stop there. If not, now repeat Step 7 again, and the data should be present.
2. <u>Nothing is connecting.</u>
At the top of the window, where it says the **"____ — AdvantageScope"**, if it says **"(Searching)"**, you are not yet connected. It should not be "Searching" for more then 10 seconds at maximum. Check to see if you are connecting to the wrong location, check if you properly configured AdvantageScope (if for Real Robot), check to see if Driver Station connected (to confirm there is an active connection.) If all fails, your firewall may be blocking NetworkT
3. <u>Connected, but nothing seems to be moving when it should be.</u> <br>
Ensure that AdvantageScope is playing in Real Time. Ensure Real Time playback is selected by clicking the circle with two rightward pointing triangles (like a fast-forward icon) in the top-right side of the window. It is enabled when it is red, and disabled when it is gray.
4. <u>The entire display seems to suddenly jitter a lot, and then go back to normal.</u> <br>
If you hover over the timeline (the moving time bar on the top of the window) when it is not playing in Real Time (see Common Errors 3,) it will show what was occuring at that time, which if you accidentally move the mouse over the timeline, can make it seem likes its jittery and moving really quickly. That is not a bug, it is an intentional feature for navigating through the timeline.