## 🌐 Remote Development Setup (SSH)

To streamline our workflow, we use VS Code's Remote-SSH extension. This allows any team member to connect their personal laptop directly to the lab PC. By doing this, we can write, edit, and execute code on the robotic arm in real-time without needing to be physically stationed at the lab computer.

### 1. Lab PC Preparation
If the lab PC needs a password reset or setup for the target user profile (`er`), run the following command directly on the lab computer's terminal:

```bash
sudo passwd er
```
*(Enter the agreed-upon team password when prompted).*

### 2. Connecting Your Personal PC via VS Code
Ensure you have the **Remote - SSH** extension installed in Visual Studio Code before proceeding.

1. Open **VS Code** on your laptop.
2. Click the green **`><` (Remote Window)** icon in the bottom-left corner of the window.
3. Select **Connect to Host...** from the command palette dropdown at the top.
4. Enter the following SSH command to target the lab PC:
   ```bash
   ssh er@10.138.69.199
   ```
5. When prompted by VS Code, enter the team password.
6. Once connected, click **Open Folder** in the explorer pane to access the repository on the lab PC. You can now run the ROS nodes and Python controllers directly from your machine!
