import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string; // Changed from Svg to image string
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI',
    image: require('@site/static/img/physical_Ai.png').default,
    description: (
      <>
        Learn about AI systems grounded in physical interaction with the real world,
        where intelligence emerges from the dynamic interaction between an agent and its environment.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    image: require('@site/static/img/human_Ai.png').default, // Using the correct image
    description: (
      <>
        Discover the principles behind creating robots that interact effectively
        in human environments and social structures.
      </>
    ),
  },
  {
    title: 'Embodied Intelligence',
    image: require('@site/static/img/Embodied_Intelligence.png').default,
    description: (
      <>
        Understand how intelligence is shaped by the body&apos;s interactions with the environment,
        exploring morphological computation and sensorimotor contingencies.
      </>
    ),
  },
];

function Feature({title, image, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img src={image} className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}