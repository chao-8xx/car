import os
import random
import shutil

# ================= 配置区 =================
# 你的原始数据集路径 (里面应该是 supplies, transport, weapon)
source_dir = r"C:\Users\mechreuo\Desktop\loong\datasets\datasets_classify"
# 笔试要用的测试文件夹
test_dir = r"/test"

# **每个大类**你想随机抽取几张图片？(比如从 supplies 所有的子类里一共抽 30 张)
num_per_main_class = 100


# ==========================================

def create_test_set():
    if not os.path.exists(source_dir):
        print(f"❌ 找不到原始数据集文件夹: {source_dir}")
        return

    # 1. 获取三大类 (supplies, transport, weapon)
    main_classes = [d for d in os.listdir(source_dir) if os.path.isdir(os.path.join(source_dir, d))]

    if not main_classes:
        print("❌ 原始数据集里没有任何大类文件夹！")
        return

    print(f"🔍 发现大类: {main_classes}")
    total_copied = 0

    for main_cls in main_classes:
        main_cls_path = os.path.join(source_dir, main_cls)
        dst_main_cls_path = os.path.join(test_dir, main_cls)

        # 在测试集里建好大类文件夹
        os.makedirs(dst_main_cls_path, exist_ok=True)

        all_images_in_main = []

        # 2. 遍历大类下的所有“小类”文件夹
        sub_classes = [d for d in os.listdir(main_cls_path) if os.path.isdir(os.path.join(main_cls_path, d))]

        valid_extensions = ('.jpg', '.jpeg', '.png', '.bmp')

        # 如果有小类文件夹，就进去捞图片
        if sub_classes:
            for sub_cls in sub_classes:
                sub_cls_path = os.path.join(main_cls_path, sub_cls)
                for f in os.listdir(sub_cls_path):
                    if f.lower().endswith(valid_extensions):
                        # 把图片路径、所属小类、原文件名记录下来
                        all_images_in_main.append({
                            'path': os.path.join(sub_cls_path, f),
                            'sub_cls': sub_cls,
                            'name': f
                        })
        else:
            # 兼容模式：如果大类下没有小类，直接是图片
            for f in os.listdir(main_cls_path):
                if f.lower().endswith(valid_extensions):
                    all_images_in_main.append({
                        'path': os.path.join(main_cls_path, f),
                        'sub_cls': 'none',
                        'name': f
                    })

        # 3. 🎲 核心：在整个大类的图片池里洗牌
        actual_num = min(num_per_main_class, len(all_images_in_main))
        if actual_num == 0:
            print(f"⚠️ 大类 [{main_cls}] 里没有任何图片，跳过。")
            continue

        random.shuffle(all_images_in_main)
        selected_images = all_images_in_main[:actual_num]

        # 4. 复制并重命名防覆盖
        for img_info in selected_images:
            src_img_path = img_info['path']

            # 组合新文件名，例如：bottle_1.jpg
            if img_info['sub_cls'] != 'none':
                new_file_name = f"{img_info['sub_cls']}_{img_info['name']}"
            else:
                new_file_name = img_info['name']

            dst_img_path = os.path.join(dst_main_cls_path, new_file_name)
            shutil.copy2(src_img_path, dst_img_path)

        print(f"✅ 从 [{main_cls}] (池子共 {len(all_images_in_main)} 张) 中随机抽取了 {actual_num} 张图片.")
        total_copied += actual_num

    print("========================================")
    print(f"🎉 抽取完成！共为笔试准备了 {total_copied} 张图片。")


if __name__ == "__main__":
    create_test_set()