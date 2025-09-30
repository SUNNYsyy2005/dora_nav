#!/usr/bin/env python3
"""
Python版本的项目路径管理器
与 include/project_paths.h 保持一致
"""

import os
from pathlib import Path

def get_project_root():
    """获取项目根目录的绝对路径"""
    # 从环境变量获取项目根目录
    env_root = os.getenv("DORA_NAV_ROOT")
    if env_root:
        return Path(env_root)
    
    # 尝试从当前文件路径推导项目根目录
    current_file = Path(__file__).resolve()
    
    # 向上查找包含 "dora_nav" 的目录
    for parent in current_file.parents:
        if parent.name == "dora_nav":
            return parent
    
    # 默认回退路径
    return Path("/home/sunny/dora_nav")

def build_path(relative_path):
    """构建文件路径的辅助函数"""
    return get_project_root() / relative_path

class ProjectPaths:
    """项目路径管理类"""
    
    @staticmethod
    def build_nav_laser_data():
        return str(build_path("build/nav/laser_data.pgm"))
    
    @staticmethod
    def build_teb_path_csv():
        return str(build_path("build/teb/path.csv"))
    
    @staticmethod
    def build_slam_data():
        return str(build_path("build/slam/laser_data.dat"))
    
    @staticmethod
    def build_nav_data():
        return str(build_path("build/nav/"))
    
    @staticmethod
    def nav_data_pgm(dataset):
        return str(build_path(f"build/nav/{dataset}.pgm"))
    
    @staticmethod
    def nav_data_yaml(dataset):
        return str(build_path(f"build/nav/{dataset}.yaml"))
    
    @staticmethod
    def amcl_txt():
        return str(build_path("amcl.txt"))
    
    @staticmethod
    def teb_txt():
        return str(build_path("teb.txt"))
    
    @staticmethod
    def nav_output_pgm():
        return str(build_path("build/nav/output.pgm"))
    
    @staticmethod
    def nav_output2_pgm():
        return str(build_path("build/nav/output2.pgm"))
    
    @staticmethod
    def amcl_output_pgm():
        return str(build_path("build/amcl/output.pgm"))
    
    # 地图相关的扩展路径
    @staticmethod
    def slam_map_yaml():
        """SLAM生成的地图YAML文件"""
        return str(build_path("build/nav/slam_map.yaml"))
    
    @staticmethod
    def slam_map_pgm():
        """SLAM生成的地图PGM文件"""
        return str(build_path("build/nav/slam_map.pgm"))
    
    @staticmethod
    def simulation_map_yaml():
        """仿真地图YAML文件"""
        return str(build_path("build/simulation_map_800x800.yaml"))
    
    @staticmethod
    def get_standard_map_search_paths():
        """获取标准的地图搜索路径列表（按优先级排序）"""
        return [
            # NAV目录下的laser_data配置
            ProjectPaths.nav_data_yaml("laser_data"),
            # SLAM生成的地图（最高优先级）
            ProjectPaths.slam_map_yaml(),
            # 仿真地图
            ProjectPaths.simulation_map_yaml(),
            # 其他可能的地图位置
            str(build_path("build/map.yaml")),
            str(build_path("map.yaml")),
        ]
    
    @staticmethod
    def find_map_image_for_yaml(yaml_path):
        """根据YAML文件路径查找对应的图像文件"""
        yaml_file = Path(yaml_path)
        if not yaml_file.exists():
            return None
        
        # 从YAML文件读取图像文件名
        try:
            import yaml
            with open(yaml_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'image' in config:
                image_file = config['image']
                # 构建完整路径
                image_path = yaml_file.parent / image_file
                if image_path.exists():
                    return str(image_path)
        except Exception:
            pass
        
        # 如果YAML中没有指定或文件不存在，尝试猜测
        base_name = yaml_file.stem
        possible_extensions = ['.pgm', '.png', '.jpg', '.jpeg']
        
        for ext in possible_extensions:
            image_path = yaml_file.parent / (base_name + ext)
            if image_path.exists():
                return str(image_path)
        
        return None

# 便捷的全局访问函数
def get_project_paths():
    return ProjectPaths()

if __name__ == "__main__":
    # 测试路径管理器
    paths = ProjectPaths()
    
    print("项目根目录:", get_project_root())
    print("SLAM地图YAML:", paths.slam_map_yaml())
    print("NAV laser_data YAML:", paths.nav_data_yaml("laser_data"))
    print("标准地图搜索路径:")
    for i, path in enumerate(paths.get_standard_map_search_paths(), 1):
        exists = "✅" if Path(path).exists() else "❌"
        print(f"  {i}. {exists} {path}")
