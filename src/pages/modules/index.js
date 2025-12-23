import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function Modules() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Modules - ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Modules">
      <main>
        <div className="container padding-vert--lg">
          <div className="row">
            <div className="col col--6 col--offset-3">
              <h1 className="hero__title">Physical AI & Humanoid Robotics Modules</h1>
              <p className="hero__subtitle">Select a module to begin your learning journey</p>

              <div className="margin-vert--lg">
                <div className="card">
                  <div className="card__header">
                    <h3>Module 1: ROS 2 Middleware for Humanoid Control</h3>
                  </div>
                  <div className="card__body">
                    <p>Learn the fundamentals of ROS 2 and how to implement distributed control systems for humanoid robots.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--secondary button--block"
                      to="/modules/ros2-middleware/constitution">
                      Start Module 1
                    </Link>
                  </div>
                </div>

                <div className="card margin-vert--lg">
                  <div className="card__header">
                    <h3>Module 2: Digital Twin Simulation for Humanoid Robotics</h3>
                  </div>
                  <div className="card__body">
                    <p>Explore simulation techniques using Gazebo and Unity for developing and testing humanoid robot control algorithms.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--secondary button--block"
                      to="/modules/digital-twin/constitution">
                      Start Module 2
                    </Link>
                  </div>
                </div>

                <div className="card margin-vert--lg">
                  <div className="card__header">
                    <h3>Module 3: The AI-Robot Brain (NVIDIA Isaac™)</h3>
                  </div>
                  <div className="card__body">
                    <p>Develop advanced perception, localization, and navigation capabilities using NVIDIA Isaac technologies.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--secondary button--block"
                      to="/modules/isaac-ai-brain/constitution">
                      Start Module 3
                    </Link>
                  </div>
                </div>

                <div className="card margin-vert--lg">
                  <div className="card__header">
                    <h3>Module 4: Vision-Language-Action (VLA)</h3>
                  </div>
                  <div className="card__body">
                    <p>Integrate voice input, vision perception, and cognitive planning to enable autonomous humanoid behavior.</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--secondary button--block"
                      to="/modules/vla/constitution">
                      Start Module 4
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}