import React from 'react';
import Content from '@theme-original/DocItem/Content';
import PersonalizationCard from '@site/src/components/PersonalizationCard';
import Quiz from '@site/src/components/Quiz';
import {useDoc} from '@docusaurus/plugin-content-docs/client';

export default function ContentWrapper(props) {
  const {metadata} = useDoc();
  return (
    <>
      <div className="margin-bottom--md">
        <PersonalizationCard topic={metadata.title} />
      </div>
      <Content {...props} />
      <div className="margin-top--lg">
        <Quiz topic={metadata.title} />
      </div>
    </>
  );
}