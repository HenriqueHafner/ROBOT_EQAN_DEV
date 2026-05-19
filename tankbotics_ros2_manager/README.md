## Add the submodule
```Terminal
git submodule add https://bitbucket.org/on_dev/tankbotics_ros2_manager.git tankbotics_ros2_manager
```
This command creates the tankbotics_ros2_manager folder at the root of your repository and registers the repository as a Git submodule.

## Repository Structure

- **`intra_process_communication/`**  
  Provides the high-performance intra-process client/server communication system.  
  This is a **required dependency** for all other modules that need to publish or receive data in real-time.

- **`gamepad_interface/`**  
  Unified interface for reading modern USB game controllers (Logitech HOTAS Stick/Throttle and Xbox 360 Wireless).  
  Uses pygame (SDL2) for reliable polling and publishes normalized data (axes as floats and buttons as packed 3-byte data) through the intra-process communication system.

## Important Notes

- The **`intra_process_communication`** server **must be running** before starting any gamepad client or other modules, from **tankbotics_ros2_manager** repository.
- All gamepad modules depend on the intra-process client to publish controller states across nodes.

---

For detailed documentation of each module, see the `README.md` file inside each folder.