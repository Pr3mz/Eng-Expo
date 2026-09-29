import re

with open('auto/auto_1.3.0.py', 'r') as f:
    content = f.read()

old_res = """            roboflow_predictions  = scaled
            roboflow_gem_targets  = gems
            roboflow_drop_targets = drops

        except Exception as e:"""

new_res = """            roboflow_predictions  = scaled
            roboflow_gem_targets  = gems
            roboflow_drop_targets = drops
            # print(f"[AI] Gems: {len(gems)} | Drops: {len(drops)}")

        except Exception as e:"""

content = content.replace(old_res, new_res)

with open('auto/auto_1.3.0.py', 'w') as f:
    f.write(content)
