import os
import numpy as np
import tempfile, zipfile
import torch
import torch.nn as nn
import torch.nn.functional as F
try:
    import torchvision
    import torchaudio
except:
    pass

class Model(nn.Module):
    def __init__(self):
        super(Model, self).__init__()

        self.conv2d_0 = nn.Conv2d(bias=True, dilation=(1,1), groups=1, in_channels=3, kernel_size=(3,3), out_channels=16, padding=(1,1), padding_mode='zeros', stride=(2,2))
        self.conv2d_1 = nn.Conv2d(bias=True, dilation=(1,1), groups=16, in_channels=16, kernel_size=(3,3), out_channels=16, padding=(1,1), padding_mode='zeros', stride=(2,2))
        self.conv2d_2 = nn.Conv2d(bias=True, dilation=(1,1), groups=1, in_channels=16, kernel_size=(1,1), out_channels=32, padding=(0,0), padding_mode='zeros', stride=(1,1))
        self.conv2d_3 = nn.Conv2d(bias=True, dilation=(1,1), groups=32, in_channels=32, kernel_size=(3,3), out_channels=32, padding=(1,1), padding_mode='zeros', stride=(2,2))
        self.conv2d_4 = nn.Conv2d(bias=True, dilation=(1,1), groups=1, in_channels=32, kernel_size=(1,1), out_channels=64, padding=(0,0), padding_mode='zeros', stride=(1,1))
        self.conv2d_5 = nn.Conv2d(bias=True, dilation=(1,1), groups=64, in_channels=64, kernel_size=(3,3), out_channels=64, padding=(1,1), padding_mode='zeros', stride=(2,2))
        self.conv2d_6 = nn.Conv2d(bias=True, dilation=(1,1), groups=1, in_channels=64, kernel_size=(1,1), out_channels=128, padding=(0,0), padding_mode='zeros', stride=(1,1))
        self.F_linear_0 = nn.Linear(bias=True, in_features=128, out_features=3)

        archive = zipfile.ZipFile('model_packed.pnnx.bin', 'r')
        self.conv2d_0.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_0.bias', (16), 'float32')
        self.conv2d_0.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_0.weight', (16,3,3,3), 'float32')
        self.conv2d_1.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_1.bias', (16), 'float32')
        self.conv2d_1.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_1.weight', (16,1,3,3), 'float32')
        self.conv2d_2.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_2.bias', (32), 'float32')
        self.conv2d_2.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_2.weight', (32,16,1,1), 'float32')
        self.conv2d_3.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_3.bias', (32), 'float32')
        self.conv2d_3.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_3.weight', (32,1,3,3), 'float32')
        self.conv2d_4.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_4.bias', (64), 'float32')
        self.conv2d_4.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_4.weight', (64,32,1,1), 'float32')
        self.conv2d_5.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_5.bias', (64), 'float32')
        self.conv2d_5.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_5.weight', (64,1,3,3), 'float32')
        self.conv2d_6.bias = self.load_pnnx_bin_as_parameter(archive, 'conv2d_6.bias', (128), 'float32')
        self.conv2d_6.weight = self.load_pnnx_bin_as_parameter(archive, 'conv2d_6.weight', (128,64,1,1), 'float32')
        self.F_linear_0.bias = self.load_pnnx_bin_as_parameter(archive, 'F_linear_0.bias', (3), 'float32')
        self.F_linear_0.weight = self.load_pnnx_bin_as_parameter(archive, 'F_linear_0.weight', (3,128), 'float32')
        archive.close()

    def load_pnnx_bin_as_parameter(self, archive, key, shape, dtype, requires_grad=True):
        return nn.Parameter(self.load_pnnx_bin_as_tensor(archive, key, shape, dtype), requires_grad)

    def load_pnnx_bin_as_tensor(self, archive, key, shape, dtype):
        fd, tmppath = tempfile.mkstemp()
        with os.fdopen(fd, 'wb') as tmpf, archive.open(key) as keyfile:
            tmpf.write(keyfile.read())
        m = np.memmap(tmppath, dtype=dtype, mode='r', shape=shape).copy()
        os.remove(tmppath)
        return torch.from_numpy(m)

    def forward(self, v_0):
        v_1 = self.conv2d_0(v_0)
        v_2 = F.relu(v_1)
        v_3 = self.conv2d_1(v_2)
        v_4 = F.relu(v_3)
        v_5 = self.conv2d_2(v_4)
        v_6 = F.relu(v_5)
        v_7 = self.conv2d_3(v_6)
        v_8 = F.relu(v_7)
        v_9 = self.conv2d_4(v_8)
        v_10 = F.relu(v_9)
        v_11 = self.conv2d_5(v_10)
        v_12 = F.relu(v_11)
        v_13 = self.conv2d_6(v_12)
        v_14 = F.relu(v_13)
        v_15 = torch.mean(v_14, dim=(-1,-2), keepdim=True)
        v_16 = v_15.reshape(1, 128)
        v_17 = self.F_linear_0(v_16)
        return v_17

def export_torchscript():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 3, 64, 64, dtype=torch.float)

    mod = torch.jit.trace(net, v_0)
    mod.save("model_packed_pnnx.py.pt")

def export_onnx():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 3, 64, 64, dtype=torch.float)

    torch.onnx.export(net, v_0, "model_packed_pnnx.py.onnx", export_params=True, operator_export_type=torch.onnx.OperatorExportTypes.ONNX_ATEN_FALLBACK, opset_version=13, input_names=['in0'], output_names=['out0'])

def export_pnnx():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 3, 64, 64, dtype=torch.float)

    import pnnx
    pnnx.export(net, "model_packed_pnnx.py.pt", v_0)

def export_ncnn():
    export_pnnx()

@torch.no_grad()
def test_inference():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 3, 64, 64, dtype=torch.float)

    return net(v_0)

if __name__ == "__main__":
    print(test_inference())
