// pseudo code
// incremental mapper
void Reconstruct() {
    if (!given(initial_recon)) {
        repeat (options_->init_num_trials) {
            if (!give(options_->init_image_id1) || !give(options_->init_image_id2)) {
                echo("Finding good initial image pair");
                if (!success(mapper.FindInitialImagePair())) {
                    echo("No good initial image pair found");
                    break;
                }
            }
            echo("Initializing with image pair #%d and #%d", get_image_pair())
            if (!success(mapper.RegisterInitialImagePair())) {
                echo("Initialization failed - possible solutions: ...");
                break;
            }
            AdjustGlobalBundle();
            echo("Filtered observations: %d", FilterPoints(options.filter_max_reproj_error,
                                                           options.filter_min_tri_angle));
            echo("Filtered images: %d", FilterImages(options.min_focal_length_ratio,
                                                     options.max_focal_length_ratio,
                                                     options.max_extra_param));
            if (recon.reg_cnt == 0) {
                if (give(options_->init_image_id1) && give(options_->init_image_id2)) break;
                else continue;
            }
        
        }
    }

    Callback(INITIAL_IMAGE_PAIR_REG_CALLBACK);

    int fail_cnt = 0;
    while (fail_cnt < 2) {
        for (auto [cnt, image] : enumerate(mapper.FindNextImages())) {
            echo("Registering image #%d (%d)", image.id);
            echo("Image sees %d / %d points", image.NumVisiblePoints3D(), image.NumObservations());
            if (success(mapper.RegisterNextImage(image.id))) {
                TriangulateImage();
                IterativeLocalRefinement();
                if (needed) IterativeGlobalRefinement();
                Callback(NEXT_IMAGE_REG_CALLBACK);
                fail_cnt = 0;
                break;
            }
            echo("Could not register, trying another image");
            ++fail_cnt;
            if (cnt > kMinNumInitialRegTrials && recon.reg_cnt < options_->min_model_size)
                break;
        }
        if (mapper.NumSharedRegImages() >= options_.max_model_overlap)
            break;
        if (fail_cnt == 1) IterativeGlobalRefinement();
    }
    if (!is_global(lastRefinement)) IterativeGlobalRefinement();

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

