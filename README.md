<div align="center">

![FRC 1165 - Team Paradise Banner](/images/banner.png)
# 2025 Robot Code

</div>

# Setting Up Code Environment

> [!NOTE]
> This code repository is meant to be used in IntelliJ, but it can be used in Visual Studio Code. In VS Code, you won't be able to run code replay or run performance profiling without going through the terminal.

<details>
<summary>IntelliJ Setup</summary>

1. Install [IntelliJ IDEA Community Edition (free) or IntelliJ IDEA Ultimate (paid, free through GitHub Student Developer Pack)](https://www.jetbrains.com/idea/download). Make sure you have the latest version of WPILib 2025 installed as well.
2. Add the [FRC IntelliJ plugin](https://plugins.jetbrains.com/plugin/9405-frc). This can be installed in the IDE directly by going through File -> Settings -> Plugins.
3. Clone this repository using your favorite Git client, and open it in IntelliJ.
4. Go to File -> Project Structure, and hit the "Edit" button next to SDK. Press the "plus" icon, press "Add JDK from disk...", and go to the directory "C:\Users\Public\wpilib\2025\jdk" and add it. This ensures that we are using the officially supported JDK version from WPILib, to hopefully minimize issues.
5. Run the "Build Robot" run configuration once, just to make sure that you've downloaded all dependencies.
6. (Optional) Add the [google-java-format plugin](https://plugins.jetbrains.com/plugin/8527-google-java-format) for proper code formatting, or, just let spotless handle it when building.
7. (Optional) You should see a prompt in the bottom right corner of IntelliJ asking you to enable google-java-format, if so, click "Enable for this project". If not, go to "File -> Settings -> google-java-format settings", and enable it.
</details>
<details>
<summary>VS Code Setup</summary>

1. Install the latest version of [WPILib](https://github.com/wpilibsuite/allwpilib/releases) and it's built-in version of VS Code.
2. Clone this repository using your favorite Git client, and open it in VS Code.
3. Build the project once to make sure that all dependencies have been downloaded.
4. (Optional) Add the [google-java-format extension](https://marketplace.visualstudio.com/items?itemName=JoseVSeb.google-java-format-for-vs-code) through VS Code's built-in extension manager.
5. (Optional) When/if you format code for the first time, make sure you select "Google Java Format for VS Code". Or, don't format, and just let spotless handle it when building.
</details>

# Run Configurations
### Build Robot
<details>
<summary>IntelliJ</summary>

Just run the "Build Robot" run configuration by selecting it and hitting the play button.
</details>
<details>
<summary>VS Code</summary>

Open the Command Palette (Ctrl+Shift+P) and type ```WPILib: Build Robot Code```.
</details>
<details>
<summary>Terminal/PowerShell</summary>

Open the project folder in a Unix terminal or PowerShell, and run ```gradlew build```.
</details>

### Build & Deploy
<details>
<summary>IntelliJ</summary>

Just run the "Build & Deploy" run configuration by selecting it and hitting the play button.
</details>
<details>
<summary>VS Code</summary>

Open the Command Palette (Ctrl+Shift+P) and type ```WPILib: Deploy Robot Code```. Or, use the keyboard shortcut "Shift+F5".
</details>
<details>
<summary>Terminal/PowerShell</summary>

Open the project folder in a Unix terminal or PowerShell, and run ```gradlew deploy```.
</details>

### Build & Deploy Robot with Debugging and Performance Profiling (with VisualVM or other JMX client)
<details>
<summary>IntelliJ</summary>

Just run the "Build & Deploy Robot with Debugging and Performance Profiling" run configuration by selecting it and hitting the play button. Use IntelliJ's built-in debugger and some JMX client for performance profiling.
</details>
<details>
<summary>VS Code/Terminal/PowerShell</summary>

Open the project folder in a Unix terminal or PowerShell, and run ```gradlew deploy -PprofilingMode -PdebugMode=true```. Use any Java debugger and some JMX client for performance profiling.
</details>

### Build & Simulate
<details>
<summary>IntelliJ</summary>

Just run the "Build & Simulate" run configuration by selecting it and hitting the play button. The Glass "simgui" will open by default, but you can also use Driver Station.
</details>
<details>
<summary>VS Code</summary>

Open the Command Palette (Ctrl+Shift+P) and type ```WPILib: Simulate Robot Code```. Or, use the keyboard shortcut "Shift+F5". You can select between "simgui", Driver Station, or both.
</details>
<details>
<summary>Terminal/PowerShell</summary>

Open the project folder in a Unix terminal or PowerShell, and run ```gradlew simulateJava```.
</details>

### Build & Replay
<details>
<summary>IntelliJ</summary>

Just run the "Build & Replay" run configuration by selecting it and hitting the play button. Use [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) to load and replay any log files.
</details>
<details>
<summary>VS Code/Terminal/PowerShell</summary>

Open the project folder in a Unix terminal or PowerShell, and run ```gradlew simulateJava -PreplayMode```. Use [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) to load and replay any log files.
</details>
This is Gatlin trying to test the slack bot?
