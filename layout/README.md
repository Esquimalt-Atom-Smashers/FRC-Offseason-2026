# Elastic
Elastic is a modern FRC Dashboard for displaying data.

**Note:** <br>
It is sized to one specific screen size. If your screen size is different, it may not fit. Elastic does not have options to zoom out, and the window will only extend to the size of your screen. If it does not fit, you may have to create your own Elastic layout.

# Setup
1. Open Elastic
2. Navigate to **File** → **Open Layout**
3. Navigate to the [layout](/layout/) directory
4. Open [elastic-layout.json](/layout/elastic-layout.json)
5. Navigate to **Settings** → **Network**
6. Enter the Team Number, and in IP Address Mode, open the dropdown and select **"Driver Station"**

## Additional steps after importing
1. Navigate to **Settings** → **Appearance**
2. Set Team Colour to #7122DD
3. Set Theme to "High Contrast"
4. Move window to desired position and select "Remember Window Position"
5. Enable Lock Layout


## Common Errors
Here are some common errors and how to resolve them.

1. <u>In the Real Robot, it is unable to connect.</u> <br>
Ensure your Driver Station is opened and configured correctly. If after that, and reopening Elastic and it still does not work, navigate to Settings → Network, open the dropdown and select **"Team Number"**. *Note: It is unable to connect to the Simulator in this configuration.*
2. <u>In the Simulation, it is unable to connect.</u> <br>
Ensure your Driver Station is opened and configured correctly. If after that, and reopening Elastic and it still does not work, navigate to Settings → Network, open the dropdown and select **"localhost"**. *Note: It is unable to connect to the Real Robot in this configuration.*

