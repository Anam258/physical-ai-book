import React from 'react';
import NavbarContent from '@theme-original/Navbar/Content'
import NavbarAuthButton from '@site/src/components/Auth/NavbarAuthButton'; 
export default function NavbarContentWrapper(props) {
  return (
    <>
      <NavbarContent {...props} />
      <div className="navbar-auth-button-wrapper">
        <NavbarAuthButton />
      </div>
    </>
  );
}