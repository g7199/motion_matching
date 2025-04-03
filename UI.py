import imgui
from bvh_controller import connect
from utils import blend_color

def draw_control_panel(state, viewport):
    panel_height = int(viewport.work_size.y * 0.25)
    panel_width = viewport.work_size.x

    # 중요 수정사항: 매 프레임 강제로 항상 재설정
    imgui.set_next_window_position(
        viewport.work_pos.x, 
        viewport.work_pos.y + viewport.work_size.y - panel_height,
        condition=imgui.ALWAYS
    )
    imgui.set_next_window_size(
        panel_width, panel_height, condition=imgui.ALWAYS
    )

    imgui.begin("Control Panel", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE)

    if state.get('motions'):
        for idx, motion in enumerate(state['motions']):
            imgui.text(motion['name'])
            changed, frame_val = imgui.slider_int(f"Frame##{idx}", motion['frame_idx'] + 1, 0, motion['frame_len'])
            if changed:
                motion['frame_idx'] = frame_val - 1
            changed, visible = imgui.checkbox(f"Visible##{idx}", motion.get('visible', True))
            if changed:
                motion['visible'] = visible
            imgui.separator()
    else:
        imgui.text("No motion loaded.")

    if imgui.button("Play/Pause", width=100, height=30):
        state['stop'] = not state['stop']

    imgui.end()


def draw_side_panel(state, viewport):
    side_width = int(viewport.work_size.x * 0.25)
    side_height = int(viewport.work_size.y * 0.75)  # 컨트롤 패널을 제외한 영역

    # 중요 수정사항: 매 프레임 강제로 항상 재설정
    imgui.set_next_window_position(
        viewport.work_pos.x + viewport.work_size.x - side_width, 
        viewport.work_pos.y,
        condition=imgui.ALWAYS
    )
    imgui.set_next_window_size(
        side_width, side_height, condition=imgui.ALWAYS
    )

    imgui.begin("Side Panel", flags=imgui.WINDOW_NO_TITLE_BAR | imgui.WINDOW_NO_RESIZE)

    if imgui.button("Load BVH File", width=120, height=30):
        state['open_file_dialog'] = True

    if state.get('motions'):
        imgui.separator()
        imgui.text("Loaded Motions:")
        for idx, motion in enumerate(state['motions']):
            imgui.text(f"{idx}: {motion['name']}")

        if len(state['motions']) >= 2:
            imgui.separator()
            imgui.text("Connect Motions")
            state.setdefault('connect_motion_a', 0)
            state.setdefault('connect_motion_b', 1 if len(state['motions']) > 1 else 0)

            _, motion_a_idx = imgui.combo(
                "Motion A", state['connect_motion_a'], [m['name'] for m in state['motions']])
            state['connect_motion_a'] = motion_a_idx

            _, motion_b_idx = imgui.combo(
                "Motion B", state['connect_motion_b'], [m['name'] for m in state['motions']])
            state['connect_motion_b'] = motion_b_idx

            if imgui.button("Connect Selected Motions", width=200, height=30):
                if state['connect_motion_a'] != state['connect_motion_b']:
                    motion_a = state['motions'][state['connect_motion_a']]['motion']
                    motion_b = state['motions'][state['connect_motion_b']]['motion']
                    root_a = state['motions'][state['connect_motion_a']]['root']
                    connected_motion = connect(motion_a[:-200], motion_b[200:])
                    new_name = f"Connected({state['motions'][state['connect_motion_a']]['name']}+" \
                               f"{state['motions'][state['connect_motion_b']]['name']})"
                    new_entry = {
                        'name': new_name,
                        'root': root_a,
                        'motion': connected_motion,
                        'frame_len': connected_motion.frames,
                        'visible': True,
                        'frame_idx': 0,
                        'color': blend_color(state['motions'][state['connect_motion_a']]['color'], state['motions'][state['connect_motion_b']]['color'])
                    }
                    state['motions'].append(new_entry)
                    print("Motions connected:", new_name)

    imgui.end()
