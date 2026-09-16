import os

# 1. 配置路径（基于你截图里的位置）
image_folder = r"E:\code\Dataset_Split_mytrain\int8\images"
txt_path = r"E:\code\Dataset_Split_mytrain\int8\imagelist.txt"

count = 0
with open(txt_path, "w", encoding="utf-8") as f:
    for img_name in os.listdir(image_folder):
        # 过滤出图片文件
        if img_name.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
            # NCNN 需要相对路径，因为我们在外层运行命令，所以路径是 int8/images/文件名
            f.write(f"int8/images/{img_name}\n")
            count += 1

print(f"✅ 搞定！成功将 {count} 张图片的路径写入了 imagelist.txt")