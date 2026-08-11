            # ============ 自行车专用优化分支 ============
            if cid == 1:  # 自行车
                # 1. 使用动态宽度，但限制范围防止异常
                dL_ground, _ = self._intersect_ray_with_plane_z(cam_cx=cam_cx, cam_cy=cam_cy, cam_cz=cam_cz, ray_dir=dL, z_plane=0.0,)
                dR_ground, _ = self._intersect_ray_with_plane_z(cam_cx=cam_cx, cam_cy=cam_cy, cam_cz=cam_cz, ray_dir=dR, z_plane=0.0,)
                
                if dL_ground is not None and dR_ground is not None:
                    xL, yL, _ = dL_ground
                    xR, yR, _ = dR_ground
                    dyn_w = math.hypot(xR - xL, yR - yL)
                    
                    # 自行车宽度限制：0.3m ~ 0.8m（正常自行车宽度）
                    min_bike_w = 0.3
                    max_bike_w = 0.8
                    if math.isfinite(dyn_w) and min_bike_w <= dyn_w <= max_bike_w:
                        w_use = dyn_w
                    else:
                        # 超出范围使用先验宽度，但可适当调整
                        w_use = min(max(w, min_bike_w), max_bike_w)
                else:
                    w_use = w
          
                l_use = l
                
                # 3. 中心位置优化：使用左右射线中点和上下边界
                # 使用掩码信息（如果有）获取更准确的底部中心
                if masks is not None and i < masks.shape[0]:
                    mask_i = np.asarray(masks[i])
                    if mask_i.size > 0:
                        # 找到mask的最底部区域
                        bottom_points = np.where(mask_i)
                        if len(bottom_points[0]) > 0:
                            # 取底部1/4区域的中心
                            bottom_y = max(bottom_points[0])
                            bottom_mask = (bottom_points[0] >= bottom_y) #- mask_i.shape[0] // 4)
                            if np.any(bottom_mask):
                                bottom_xs = bottom_points[1][bottom_mask]
                                bottom_ys = bottom_points[0][bottom_mask]
                                center_u = np.mean(bottom_xs)
                                center_v = np.mean(bottom_ys)
                                
                                # 使用掩码中心点计算位置
                                d_center = self._world_ray_dir(center_u, center_v)
                                p_center, _ = self._intersect_ray_with_plane_z(
                                    cam_cx=cam_cx, 
                                    cam_cy=cam_cy, 
                                    cam_cz=cam_cz, 
                                    ray_dir=d_center, 
                                    z_plane=0.0,
                                )
                                if p_center is not None:
                                    cx, cy, _ = p_center
                                else:
                                    # fallback到射线中点
                                    d_mid_ground, _ = self._intersect_ray_with_plane_z(
                                        cam_cx=cam_cx, 
                                        cam_cy=cam_cy, 
                                        cam_cz=cam_cz, 
                                        ray_dir=d_mid, 
                                        z_plane=0.0,
                                    )
                                    if d_mid_ground is not None:
                                        cx, cy, _ = d_mid_ground
                                    else:
                                        continue
                            else:
                                continue
                        else:
                            continue
                    else:
                        # 无有效掩码，使用射线中点
                        d_mid_ground, _ = self._intersect_ray_with_plane_z(
                            cam_cx=cam_cx, 
                            cam_cy=cam_cy, 
                            cam_cz=cam_cz, 
                            ray_dir=d_mid, 
                            z_plane=0.0,
                        )
                        if d_mid_ground is not None:
                            cx, cy, _ = d_mid_ground
                        else:
                            continue
                else:
                    # 无掩码，使用射线中点
                    d_mid_ground, _ = self._intersect_ray_with_plane_z(
                        cam_cx=cam_cx, 
                        cam_cy=cam_cy, 
                        cam_cz=cam_cz, 
                        ray_dir=d_mid, 
                        z_plane=0.0,
                    )
                    if d_mid_ground is not None:
                        cx, cy, _ = d_mid_ground
                    else:
                        continue

                # 4. 高度优化：使用跟踪信息（如果有）
                h_use = h
                if track_ids is not None and i < len(track_ids):
                    track_id = int(track_ids[i])
                    if track_id > 0:
                        # 从跟踪历史中获取稳定高度
                        if hasattr(self, '_track_height_history'):
                            if track_id in self._track_height_history:
                                # 使用平滑后的高度
                                h_smooth = self._track_height_history[track_id]
                                h_use = 0.7 * h + 0.3 * h_smooth
                
                # 5. z_base估算：结合先验和掩码
                z_base = z_plane_prior
                if masks is not None and i < masks.shape[0]:
                    mask_i = np.asarray(masks[i])
                    if mask_i.size > 0:
                        # 从掩码底部估算z
                        bottom_pts = np.where(mask_i)
                        if len(bottom_pts[0]) > 0:
                            bottom_y = max(bottom_pts[0])
                            # 取最底部像素
                            bottom_mask = (bottom_pts[0] == bottom_y)
                            if np.any(bottom_mask):
                                bottom_xs = bottom_pts[1][bottom_mask]
                                u_bottom = np.mean(bottom_xs)
                                v_bottom = float(bottom_y)
                                
                                # 使用mask底部点估算z
                                d_bottom = self._world_ray_dir(u_bottom, v_bottom)
                                # 假设自行车在地面上，用射线与z=0平面求交
                                p_bottom, _ = self._intersect_ray_with_plane_z(
                                    cam_cx=cam_cx, 
                                    cam_cy=cam_cy, 
                                    cam_cz=cam_cz, 
                                    ray_dir=d_bottom, 
                                    z_plane=0.0,
                                )
                                if p_bottom is not None:
                                    # 使用交点位置修正z_base
                                    _, _, z_est = p_bottom
                                    if math.isfinite(z_est):
                                        z_base = 0.5 * z_base + 0.5 * z_est

                # 6. 最终输出
                cuboids[i, 0] = float(cx)
                cuboids[i, 1] = float(cy)
                cuboids[i, 2] = float(z_base + 0.5 * h_use)
                cuboids[i, 3] = float(l_use)
                cuboids[i, 4] = float(w_use)
                cuboids[i, 5] = float(h_use)
                
                continue
            # ============ 其他类别（包括默认的ground0_dynamic_width）============
