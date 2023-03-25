// pseudo code

void IncrementalMapperController::Run() {
    client = new TCPClient();

    IncrementalMapperController::Reconstruct();
    kNumInitRelaxations = 2;
    repeat (kNumInitRelaxations) {
        if (recon.reg_cnt > 0) break;
        if (!quit(client.RelaxAndRestart(
            options_->init_min_num_inliers,
            options_->init_min_tri_angle
        ))) break;
        options_->init_min_num_inliers /= 2;
        IncrementalMapperController::Reconstruct();

        if (recon.reg_cnt > 0) break;
        if (!quit(client.RelaxAndRestart(
            options_->init_min_num_inliers,
            options_->init_min_tri_angle
        ))) break;
        options_->init_min_tri_angle /= 2;
        IncrementalMapperController::Reconstruct();
    }
}


void IncrementalMapperController::Reconstruct() {
    for (num_trials : options_->init_num_trials) {
        // 
        client.BeginReconstruction(num_trials);
        if (!given(initial_recon)) {
            image_id1 = options_->init_image_id1;
            image_id2 = options_->init_image_id2;
            client.FindInitialImagePair(image_id1, image_id2);
            if (!give(image_id1) || !give(image_id2)) {
                echo("Finding good initial image pair");
                if (!success(mapper.FindInitialImagePair())) {
                    echo("No good initial image pair found");
                    client.EndReconstruction(num_trials, 0, 0);
                    break;
                }
                if (!exist(image_id1) || !exist(image_id2)) {
                    client.EndReconstruction(num_trials, 0, 0);
                    return;
                }
            }
            client.InitializeWithImagePair(image_id1, image_id2)
            echo("Initializing with image pair #%d and #%d", image_id1, image_id2)
            if (!success(mapper.RegisterInitialImagePair())) {
                echo("Initialization failed - possible solutions: ...");
                client.FailInitialization(
                    image_id1, image_id2,
                    init_mapper_options.init_min_tri_angle,
                    init_mapper_options.init_min_num_inliers
                );
                client.EndReconstruction(num_trials, 0, 0);
                break;
            }
            if (options_->ba_global) AdjustGlobalBundle();
            echo("Filtered observations: %d", FilterPoints(options.filter_max_reproj_error,
                                                           options.filter_min_tri_angle));
            echo("Filtered images: %d", FilterImages(options.min_focal_length_ratio,
                                                     options.max_focal_length_ratio,
                                                     options.max_extra_param));
            if (!recon.NumRegImages() || !recon.NumPoints3D()) {
                client.FailInitialRegistration(image_id1, image_id2, recon.NumRegImages(), recon.NumPoints3D());
                client.FailDueToLittleTriAngle(image_id1, image_id2);
                client.EndReconstruction(num_trials, recon.NumRegImages(), recon.NumPoints3D());
                if (give(options_->init_image_id1) && give(options_->init_image_id2)) break;
                else continue;
            }

            client.SucceedInitialRegistration(image_id1, image_id2, recon.NumRegImages(), recon.NumPoints3D());
        }
    }

    Callback(INITIAL_IMAGE_PAIR_REG_CALLBACK);

    fail_cnt = 0;
    while (fail_cnt < 2) {
        next_images = mapper.FindNextImages();
        client.FindNextImages(next_images);
        for ([cnt, image] : enumerate(next_images)) {
            client.RegisterNextImage(image.id, recon.NumRegImages());
            echo("Registering image #%d (%d)", image.id);
            echo("Image sees %d / %d points", image.NumVisiblePoints3D(), image.NumObservations());
            if (success(mapper.RegisterNextImage(image.id))) {
                TriangulateImage();
                IterativeLocalRefinement();
                if (options_->ba_global && needed) IterativeGlobalRefinement();
                client.SucceedRegistration(image.id, recon.NumRegImages(), recon.NumPoints3D())
                Callback(NEXT_IMAGE_REG_CALLBACK);
                fail_cnt = 0;
                break;
            }
            client.FailRegistration(image.id, cnt, recon.NumRegImages());
            if (client.GiveUp()) break;
            echo("Could not register, trying another image");
            ++fail_cnt;
            if (cnt > kMinNumInitialRegTrials && recon.reg_cnt < options_->min_model_size)
                break;
        }
        if (mapper.NumSharedRegImages() >= options_.max_model_overlap)
            break;
        client.FailAllRegistration(fail_cnt);
        if (fail_cnt == 1 && options_->ba_global) IterativeGlobalRefinement();
    }
    if (!is_global(lastRefinement) && options_->ba_global) IterativeGlobalRefinement();

    client.EndReconstruction(num_trials, recon.NumRegImages(), recon.NumPoints3D());
    Callback(LAST_IMAGE_REG_CALLBACK);
}


bool mapper::RegisterInitialImagePair() {
    auto [image_id1, image_id2] = get_image_pair();
    if (!EstimateInitialTwoViewGeometry(image_id1, image_id2)) {
        return false;
    }
    reconstruction_->RegisterImage(image_id1);
    reconstruction_->RegisterImage(image_id2);
    RegisterImageEvent(image_id1);
    RegisterImageEvent(image_id2);
    auto corrs = correspondence_graph.FindCorrespondencesBetweenImages(image_id1, image_id2);

    Track track;
    track[0].image_id = image_id1;
    track[1].image_id = image_id2;
    for (const auto& corr : corrs) {
        Vector2d point1_N = camera1.ImageToWorld(image1.Point2D(corr.point2D_idx1).XY());
        Vector2d point2_N = camera2.ImageToWorld(image2.Point2D(corr.point2D_idx2).XY());
        Vector3d xyz = TriangulatePoint(proj_matrix1, proj_matrix2, point1_N, point2_N);
        double tri_angle = CalculateTriangulationAngle(proj_center1, proj_center2, xyz);
        if (tri_angle >= options.init_min_tri_angle &&
            HasPointPositiveDepth(proj_matrix1, xyz) &&
            HasPointPositiveDepth(proj_matrix2, xyz))
        {
            track[0].point2D_idx = corr.point2D_idx1;
            track[1].point2D_idx = corr.point2D_idx2;
            reconstruction_->AddPoint3D(xyz, track);
        }
    }
    return true;
}

