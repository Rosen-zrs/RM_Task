import torch
import torch.nn as nn
from torch import optim
from torch.autograd import Variable
from torch.utils.data import DataLoader
from torchvision import transforms
from torchvision import datasets
from tqdm import tqdm
from VGG_16_model import VGG_16

BATCH_SIZE = 100

# 下载训练集 CIFAR-10 10分类训练集
train_dataset = datasets.CIFAR10('./data', train=True, transform=transforms.ToTensor(), download=True)
train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
test_dataset = datasets.CIFAR10('./data', train=False, transform=transforms.ToTensor(), download=True)
test_loader = DataLoader(test_dataset, batch_size=BATCH_SIZE, shuffle=False)

#实例化
model = VGG_16()

#加载模型参数
model.load_state_dict(torch.load('bz_512_lr_0.005.pkl', map_location='cpu'))
model.eval()


torch.no_grad()

criterion = nn.CrossEntropyLoss()

# 测试模型
eval_loss = 0
eval_acc = 0
for data in test_loader:  
    img, label = data

    img = Variable(img)
    label = Variable(label)

    out = model(img)

    loss = criterion(out, label)
    eval_loss += loss.item() * label.size(0)

    _, pred = torch.max(out, 1)

    num_correct = (pred == label).sum()
    eval_acc += num_correct.item()

    print("one more")

print('Test Loss: {:.6f}, Acc: {:.6f}'.format(eval_loss / (len(test_dataset)), eval_acc / (len(test_dataset))))
print()