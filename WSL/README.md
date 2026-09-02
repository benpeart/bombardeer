This complete end-to-end guide details how to take a fresh WSL2 environment on Windows, set up the Dockerized Hailo AI Software Suite with NVIDIA GPU acceleration, optimize MegaDetector v6 (YOLOv9-c), and compile the binary `MDV6-yolov9-c.hef` for execution on the Raspberry Pi 5 Hailo-8 NPU.

---

### Prerequisites & Architecture

* **Host Machine:** Windows 10/11 with WSL2 (Ubuntu), NVIDIA GPU (e.g., RTX 40-series), and current NVIDIA drivers installed on Windows.
* **Target Hardware:** Raspberry Pi 5 with Hailo-8 M.2 AI HAT.
* **Model:** MegaDetector v6 Compact (`MDV6-yolov9-c.onnx`, $640\times640$ input, 3 classes: Animal, Person, Vehicle).

---

### Step 1: Prepare the WSL2 Host Environment

Open PowerShell as Administrator to ensure WSL2 is up to date:

```powershell
wsl --update

```

Open your WSL Ubuntu terminal and install Docker CE, the NVIDIA Container Toolkit, and build tools:

```bash
# 1. Update system packages
sudo apt update && sudo apt install -y curl gnupg lsb-release wget ca-certificates

# 2. Install Docker Engine
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 3. Add current user to Docker group & start service
sudo usermod -aG docker $USER
sudo service docker start

# 4. Install NVIDIA Container Toolkit (for GPU passthrough into Docker)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo service docker restart

```

Verify GPU availability within Docker:

```bash
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

```

---

### Step 2: Set Up the Persistent Shared Workspace

Create a dedicated workspace folder on your WSL host that will remain mounted inside the container:

```bash
mkdir -p ~/hailo/shared_with_docker
cd ~/hailo

```

Download the official Hailo AI Software Suite Docker image archive (e.g., `hailo8_ai_sw_suite_2026-07.zip` or `.tar.gz`) from the [Hailo Developer Zone](https://hailo.ai/developer-zone/) into `~/hailo`.

Load the Docker image (assuming the extracted image tar is present):

```bash
docker load -i hailo8_ai_sw_suite_2026-07.tar

```

---

### Step 3: Run the GPU-Accelerated Hailo Suite Container

Launch the container with full compute/utility driver capabilities and mount your shared directory:

```bash
docker run --gpus 'all,"capabilities=compute,utility"' \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --privileged --net=host -e DISPLAY=:0 \
  --ipc=host --group-add 44 \
  -v /dev:/dev -v /lib/firmware:/lib/firmware -v /lib/modules:/lib/modules \
  -v /lib/udev/rules.d:/lib/udev/rules.d -v /usr/src:/usr/src \
  --name hailo8_ai_sw_suite_container \
  -v $HOME/hailo/shared_with_docker/:/local/shared_with_docker:rw \
  -ti hailo8_ai_sw_suite_2026-07:latest

```

---

### Step 4: Configure Container Environment & cuDNN

Inside the container shell `(hailo_virtualenv) hailo@...:/local/workspace$`:

1. **Configure dynamic library and GPU ID variables:**
```bash
cat << 'EOF' >> ~/.bashrc
export CUDNN_PATH=$(python3 -c "import nvidia.cudnn; print(list(nvidia.cudnn.__path__)[0] + '/lib')" 2>/dev/null)
export LD_LIBRARY_PATH=$CUDNN_PATH:$LD_LIBRARY_PATH
export CUDA_VISIBLE_DEVICES=0
export HAILO_GPU_ID=0
EOF

source ~/.bashrc

```


2. **Verify GPU detection:**
```bash
hailo --version

```


*(Ensure no fallback warnings appear and cuBLAS/cuDNN initialize).*
3. **Navigate to the shared directory:**
```bash
cd /local/shared_with_docker

```



---

### Step 5: Acquire the ONNX Model & Generate Calibration Data

1. **Download the official MegaDetector v6 YOLOv9-c ONNX model:**
```bash
wget -O MDV6-yolov9-c.onnx "https://zenodo.org/records/15398270/files/MDV6-yolov9-c.onnx?download=1"

```


2. **Generate Calibration Dataset (`calib_data.npy`):**
The Dataflow Compiler requires a calibration set matching the input layer dimensions: 64 to 128 images of shape `(N, 640, 640, 3)` with `uint8` values $[0, 255]$.
```bash
python3 -c "
import numpy as np
# Generate 64 calibration frames matching 640x640x3 RGB
calib = np.random.randint(0, 256, size=(64, 640, 640, 3), dtype=np.uint8)
np.save('calib_data.npy', calib)
print('Created calib_data.npy:', calib.shape, calib.dtype)
"

```


*(Note: You can replace random noise with real pasture/camera images resized to $640\times640\times3$ for marginally tighter quantization SNR).*

---

### Step 6: Parse the ONNX Model into Hailo Archive (HAR)

Run the parser with an explicit input shape. Supply `"y"` to accept the recommended YOLO detection heads:

```bash
printf "y\ny\n" | hailo parser onnx MDV6-yolov9-c.onnx \
  --hw-arch hailo8 \
  --start-node-names images \
  --tensor-shapes [1,3,640,640] \
  --har-path MDV6_yolov9_c.har

```

This extracts the three scale heads:

* Stride 8: `conv98` ($80\times80\times64$) & `conv100` ($80\times80\times3$)
* Stride 16: `conv121` ($40\times40\times64$) & `conv122` ($40\times40\times3$)
* Stride 32: `conv143` ($20\times20\times64$) & `conv144` ($20\times20\times3$)

---

### Step 7: Quantize and Optimize

Create the compiler script configuring input normalization and maximum performance tuning:

```bash
cat << 'EOF' > mdv6.alls
normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])
performance_param(compiler_optimization_level=max)
EOF

```

Run the quantizer (leveraging the GPU for cuDNN bias correction and SNR noise analysis):

```bash
hailo optimize MDV6_yolov9_c.har \
  --calib-set-path calib_data.npy \
  --model-script mdv6.alls \
  --output-har-path MDV6_yolov9_c_quantized.har

```

---

### Step 8: Compile to Hardware Execution Format (HEF)

Run the compiler to execute graph partitioning and cluster resource allocation:

```bash
hailo compiler MDV6_yolov9_c_quantized.har

```

* Graph partitioning runs across CPU cores using an Integer Linear Programming (ILP) solver.
* The compiler will split the model into **4 contexts** to map within the Hailo-8's on-chip SRAM clusters without DDR thrashing.
* Upon completion, `MDV6-yolov9-c.hef` will be written directly to `/local/shared_with_docker/`.

---

### Step 9: Inspect the Resulting Binary

Verify the compiled binary stream configuration:

```bash
hailortcli parse-hef /local/shared_with_docker/MDV6-yolov9-c.hef

```

Confirm the following output structure:

* **Architecture:** `HAILO8`
* **Contexts:** `4`
* **Input:** `MDV6-yolov9-c/input_layer1 UINT8, NHWC(640x640x3)`
* **Outputs:** 6 regression/class tensors across strides 8, 16, and 32.

---

### Step 10: Deploy and Benchmark on the Raspberry Pi 5

Exit the container to return to your WSL host prompt:

```bash
exit

```

Transfer the HEF from your persistent directory to the Pi:

```bash
scp ~/hailo/shared_with_docker/MDV6-yolov9-c.hef ben@turret.local:~/Bombardeer/

```

SSH into your Pi 5 and run the hardware benchmark:

```bash
ssh ben@turret.local "hailortcli benchmark ~/Bombardeer/MDV6-yolov9-c.hef"

```

Expected operational baseline:

* **Throughput:** ~**24.66 FPS**
* **Hardware Latency:** ~**37.7 ms**
* **Inference Mode:** Multi-context streaming across PCIe Gen 3.