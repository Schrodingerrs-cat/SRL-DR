class ILCController {
private:
    int numPoints;
    double learningRate;
    double systemErrorLevel;
    int iteration;
    bool enableNoise;
    double smoothingAlpha;  // Smoothing factor for gradual correction

    std::vector<Point2D> reference;
    std::vector<Point3D> corrections;
    std::vector<Point2D> currentTrajectory;
    std::vector<Point2D> lastErrors;
    std::vector<std::vector<Point2D>> completedPaths;

    double lastRMSError;

    // dome adaptation control
    bool domeActive = false;
    std::vector<Point2D> domeTarget;   // target cross-section to converge to
    double domeEpsilon = 1e-3;         // stop when max point distance < this
    double domeMorphStep = 0.1;        // how aggressively to morph reference toward domeTarget per iteration

    // --- Morph transition support (gradual transition when new shape is requested)
    bool morphActive = false;
    std::vector<Point2D> morphTarget;
    double morphProgress = 0.0;   // 0..1
    double morphStep = 0.08;      // progress increment per iteration (tuneable; smaller = smoother)

    // *** CHANGED: parameters controlling morph-as-error behavior
    double morphVirtualCorrectionFactor = 0.5; // how much of the morph is applied as virtual error to corrections
    double morphSmoothingMultiplier = 1.0;     // multiply smoothingAlpha for morph-driven corrections

public:
    ILCController(int nPts, double lr) 
        : numPoints(nPts), learningRate(lr), systemErrorLevel(0.0), 
          iteration(0), enableNoise(false), smoothingAlpha(0.3), lastRMSError(0.0) {
        corrections.assign(numPoints, Point3D(0, 0, 0));
        reference = ShapeGenerator::generateCircle(numPoints);
    }

    void setReference(const std::vector<Point2D>& ref) {
        if (ref.empty()) return;
        reference = ref;
        updateWorldBounds(ref);
        reference.resize(numPoints);
        std::cout << "[ILC] Reference updated (adaptive tracking continues)\n";
    }

    // Start a gradual morph toward newShape (runs over several completeIteration() calls)
    void startMorphTo(const std::vector<Point2D>& newShape, double step = 0.08) {
        if (newShape.empty()) return;
        morphTarget = newShape;
        if (morphTarget.size() != (size_t)numPoints) morphTarget.resize(numPoints, morphTarget.back());
        morphProgress = 0.0;
        morphStep = std::max(0.005, std::min(0.3, step));
        morphActive = true;
        std::cout << "[ILC] startMorphTo called (step=" << morphStep << ")\n";
    }


    // Start dome convergence: target is a full shape (same size as reference)
    void startDomeToTarget(const std::vector<Point2D>& target) {
        if (target.empty()) return;
        domeTarget = target;
        // ensure domeTarget has same point count as reference
        if (domeTarget.size() != (size_t)numPoints) domeTarget.resize(numPoints, domeTarget.back());
        domeActive = true;
        std::cout << "[ILC] Dome mode started (adaptive convergence)\n";
    }

    void stopDome() {
        domeActive = false;
        std::cout << "[ILC] Dome mode stopped\n";
    }

    bool isDomeActive() const { return domeActive; }

    // Create a 'zero-dimension' target (collapse to centroid)
    std::vector<Point2D> makeCollapsedTarget() const {
        Point2D c = ShapeGenerator::computeCentroid(reference);
        std::vector<Point2D> tgt(numPoints, c);
        return tgt;
    }

    // New: gradually transform reference shape into a new one over multiple iterations
    void morphReferenceTo(const std::vector<Point2D>& newShape, double progress) {
        // Clamp progress 0–1
        progress = std::max(0.0, std::min(1.0, progress));
        auto morphed = ShapeGenerator::morphShapes(reference, newShape, progress);
        reference = morphed;
    }

    // New: build dome layers by reducing radius each iteration
    void buildDomeLayered(double initialRadius, double finalRadius, int totalLayers) {
        double step = (initialRadius - finalRadius) / std::max(1, totalLayers - 1);
        completedPaths.clear();

        for (int i = 0; i < totalLayers; i++) {
            double currentRadius = initialRadius - i * step;
            auto layer = ShapeGenerator::generateCircle(numPoints, currentRadius);
            completedPaths.push_back(layer);
        }
        // After dome layers are generated, set the last layer as new reference
        if (!completedPaths.empty()) {
            reference = completedPaths.back();
        }
        std::cout << "[ILC] Dome layers generated (" << totalLayers << " layers)\n";
    }

    Point2D plantModel(const Point2D& command, int pathIndex) {
        Point2D output = command;

        if (systemErrorLevel > 0) {
            double theta = (2.0 * M_PI * pathIndex) / numPoints;

            // Multi-harmonic systematic error
            double radialError = systemErrorLevel * (
                0.25 * std::sin(3 * theta) +
                0.15 * std::sin(5 * theta) +
                0.10 * std::cos(2 * theta)
            );

            double phaseError = systemErrorLevel * (0.18 + 0.02 * std::sin(0.5 * theta + iteration));
            double contractionError = systemErrorLevel * (0.12 + 0.03 * std::cos(1.5 * theta));

            double radius = std::sqrt(output.x * output.x + output.y * output.y);
            double angle = std::atan2(output.y, output.x);

            double newRadius = radius * (1 - contractionError) + radialError;
            double newAngle = angle + phaseError;

            output.x = newRadius * std::cos(newAngle);
            output.y = newRadius * std::sin(newAngle);

            // Optional noise
            if (enableNoise) {
                double noiseAmp = systemErrorLevel * 0.04;
                output.x += (rand() / (double)RAND_MAX - 0.5) * noiseAmp;
                output.y += (rand() / (double)RAND_MAX - 0.5) * noiseAmp;
            }
        }

        return output;
    }

    // *** CHANGED: compute realtime error each time we sample a point (helps shape-change RMS)
    void updateRealtimeError() {
        if (currentTrajectory.empty()) return;
        double total = 0.0;
        size_t n = currentTrajectory.size();
        for (size_t i = 0; i < n && i < reference.size(); ++i) {
            double dx = reference[i].x - currentTrajectory[i].x;
            double dy = reference[i].y - currentTrajectory[i].y;
            total += dx*dx + dy*dy;
        }
        lastRMSError = std::sqrt(total / std::max<size_t>(1, n));
    }

    Point2D getCurrentPosition(int pathIndex) {
        if (pathIndex >= numPoints) pathIndex = numPoints - 1;

        Point2D refPoint = reference[pathIndex];
        Point2D correction(corrections[pathIndex].x, corrections[pathIndex].y);

        // Apply full correction (no smoothing here - that's for the update law)
        Point2D command;
        command.x = refPoint.x + correction.x;
        command.y = refPoint.y + correction.y;

        Point2D actual = plantModel(command, pathIndex);

        if ((int)currentTrajectory.size() <= pathIndex) {
            currentTrajectory.push_back(actual);
        } else {
            currentTrajectory[pathIndex] = actual;
        }

        // update realtime RMS/error based on observed currentTrajectory
        updateRealtimeError(); // *** CHANGED: keeps lastRMSError updated continuously

        return actual;
    }

    double completeIteration() {
        if (currentTrajectory.size() < (size_t)numPoints) return lastRMSError;

        std::cout << "[ILC] Completing iteration " << iteration << std::endl;

        std::vector<Point2D> trackingErrors;
        double totalError = 0;

        // Standard ILC: Calculate error from REFERENCE, not previous iteration
        for (int i = 0; i < numPoints; i++) {
            Point2D refPt = reference[i];
            Point2D actualPt = currentTrajectory[i];

            // Direct error calculation: e_k(i) = reference(i) - actual_k(i)
            Point2D error;
            error.x = refPt.x - actualPt.x;
            error.y = refPt.y - actualPt.y;
            trackingErrors.push_back(error);

            totalError += error.x * error.x + error.y * error.y;
        }

        double rmsError = std::sqrt(totalError / numPoints);
        lastRMSError = rmsError;

        // Standard ILC update with smoothing for gradual correction
        for (int i = 0; i < numPoints && i < (int)trackingErrors.size(); i++) {
            double proposedDeltaX = learningRate * trackingErrors[i].x;
            double proposedDeltaY = learningRate * trackingErrors[i].y;

            corrections[i].x += smoothingAlpha * proposedDeltaX;
            corrections[i].y += smoothingAlpha * proposedDeltaY;
            corrections[i].z += smoothingAlpha * 0.0; // placeholder for Z correction (optional)

            double correctionMag = std::sqrt(corrections[i].x * corrections[i].x +
                                            corrections[i].y * corrections[i].y);
            if (correctionMag > 1.0) {
                corrections[i].x = (corrections[i].x / correctionMag) * 1.0;
                corrections[i].y = (corrections[i].y / correctionMag) * 1.0;
            }
        }

        iteration++;
        lastErrors = trackingErrors;

        // --- If morphActive, advance morphProgress and update reference gradually ---
        if (morphActive && !morphTarget.empty()) {
            morphProgress += morphStep;
            if (morphProgress >= 1.0) {
                reference = morphTarget;
                morphActive = false;
                morphProgress = 1.0;
                std::cout << "[ILC] Morph complete (reference replaced)\n";
            } else {
                // smoothstep blending for nicer ease-in/out
                double t = morphProgress;
                double s = t*t*(3 - 2*t);
                auto morphed = ShapeGenerator::morphShapes(reference, morphTarget, s);

                // *** CHANGED: treat part of the morph as a virtual error so corrections adapt similarly to error-driven updates.
                // This makes the ILC respond to shape-change similarly to how it responds to plant error (reduces jumps).
                for (int i = 0; i < numPoints; ++i) {
                    double vx = morphed[i].x - reference[i].x; // desired delta
                    double vy = morphed[i].y - reference[i].y;

                    // Virtual "error" scaled down by morphVirtualCorrectionFactor
                    double proposedDeltaX = learningRate * (vx * morphVirtualCorrectionFactor);
                    double proposedDeltaY = learningRate * (vy * morphVirtualCorrectionFactor);

                    corrections[i].x += smoothingAlpha * morphSmoothingMultiplier * proposedDeltaX;
                    corrections[i].y += smoothingAlpha * morphSmoothingMultiplier * proposedDeltaY;

                    double correctionMag = std::sqrt(corrections[i].x * corrections[i].x +
                                                    corrections[i].y * corrections[i].y);
                    if (correctionMag > 1.0) {
                        corrections[i].x = (corrections[i].x / correctionMag) * 1.0;
                        corrections[i].y = (corrections[i].y / correctionMag) * 1.0;
                    }
                }

                // finally update the reference gradually (so the visual reference moves smoothly)
                reference = morphed;
                // keep bounds updated for the visualizer
                updateWorldBounds(reference);
                std::cout << "[ILC] Morph progress: " << std::fixed << std::setprecision(3) << morphProgress << "\n";
            }
        }

        // --- If dome mode is active, morph the reference gradually toward domeTarget ---
        if (domeActive && !domeTarget.empty()) {
            // compute max distance to target
            double maxDist = 0.0;
            for (int i = 0; i < numPoints; ++i) {
                double dx = domeTarget[i].x - reference[i].x;
                double dy = domeTarget[i].y - reference[i].y;
                double d = std::sqrt(dx*dx + dy*dy);
                maxDist = std::max(maxDist, d);
            }

            if (maxDist <= domeEpsilon) {
                domeActive = false;
                std::cout << "[ILC] Dome target reached (maxDist=" << maxDist << ") — dome mode off\n";
            } else {
                // morph amount scales with learningRate and last RMS error so it slows if converged
                double step = std::min(0.5, std::max(0.02, learningRate * smoothingAlpha));
                // move reference points toward domeTarget by a smoothstep of step
                for (int i = 0; i < numPoints; ++i) {
                    reference[i].x = reference[i].x + step * (domeTarget[i].x - reference[i].x);
                    reference[i].y = reference[i].y + step * (domeTarget[i].y - reference[i].y);
                }
                // update bounds for visualization
                updateWorldBounds(reference);
                std::cout << "[ILC] Dome morph step applied (maxDist=" << maxDist << ", step=" << step << ")\n";
            }
        }

        std::cout << "[ILC] Iteration " << iteration-1 << " RMS Error: " << rmsError << std::endl;

        // Store completed path
        completedPaths.push_back(currentTrajectory);
        // keep a larger history so renderer can display many layers
        if (completedPaths.size() > 200) {
            completedPaths.erase(completedPaths.begin());
        }

        currentTrajectory.clear();

        if (iteration > 1) {
            std::cout << "[ILC] Adapting to possibly new reference — RMS Error now: " 
                      << lastRMSError << std::endl;
        }
        return rmsError;
    }

    void induceError(double level) {
        systemErrorLevel = level;
        std::cout << "[ILC] System error set to " << (level * 100) << "%" << std::endl;
    }

    void setLearningRate(double lr) {
        learningRate = lr;
        std::cout << "[ILC] Learning rate set to " << lr << std::endl;
    }

    void setSmoothingFactor(double alpha) {
        smoothingAlpha = std::max(0.1, std::min(1.0, alpha));
        std::cout << "[ILC] Smoothing factor set to " << smoothingAlpha << std::endl;
        std::cout << "      (Controls rate of correction update - higher = faster learning)" << std::endl;
    }

    void setNoise(bool enabled) {
        enableNoise = enabled;
        std::cout << "[ILC] Noise " << (enabled ? "enabled" : "disabled") << std::endl;
    }

    void reset() {
        corrections.assign(numPoints, Point3D(0, 0, 0));
        currentTrajectory.clear();
        lastErrors.clear();
        completedPaths.clear();
        iteration = 0;
        systemErrorLevel = 0;
        lastRMSError = 0;
        std::cout << "[ILC] Reset complete" << std::endl;
    }

    // Getters
    const std::vector<Point2D>& getReference() const { return reference; }
    const std::vector<Point2D>& getCurrentTrajectory() const { return currentTrajectory; }
    const std::vector<std::vector<Point2D>>& getCompletedPaths() const { return completedPaths; }
    const std::vector<Point2D>& getLastErrors() const { return lastErrors; }
    int getIteration() const { return iteration; }
    double getRMSError() const { return lastRMSError; }
    double getErrorLevel() const { return systemErrorLevel; }
    bool getNoiseEnabled() const { return enableNoise; }
    const std::vector<Point2D>& getBaseCrossSection() const {
        return reference;
    }

    double getAvgCorrection() const {
        double sum = 0;
        for (const auto& c : corrections) {
            sum += std::sqrt(c.x*c.x + c.y*c.y);
        }
        return sum / numPoints;
    }
};
