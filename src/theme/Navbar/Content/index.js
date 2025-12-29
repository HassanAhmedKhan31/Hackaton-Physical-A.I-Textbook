import React from 'react';
import NavbarContentOriginal from '@theme-original/Navbar/Content';
// import UserMenu from '@site/src/components/Auth/UserMenu'; // 👈 COMMENT THIS OUT

export default function NavbarContentWrapper(props) {
  return (
    <>
      <NavbarContentOriginal {...props} />
      <div className="navbar__item">
         {/* <UserMenu /> */}
      </div>
    </>
  );
}