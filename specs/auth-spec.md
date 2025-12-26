# Auth & Personalization Specification

## Context
We are implementing Requirement #5 (Better-Auth) and #6 (Personalization). We need a dedicated Node.js auth server because Better-Auth is a TypeScript library.

## Architecture
- **Auth Server:** Hono (Node.js) + Better-Auth.
- **Database:** Neon Postgres (Same DB as the Chatbot).
- **Frontend:** Docusaurus + Better-Auth React Client.

## Deliverables

### 1. `auth-server/` (The Auth Microservice)
- Initialize a Node.js project (`package.json`).
- Install `better-auth`, `hono`, `@better-auth/hono-adapter`, `pg`.
- **Schema:** Configure Better-Auth to use the Neon DB URL.
- **Custom Fields:** Extend the `user` schema to include:
  - `background`: Enum ("software", "hardware", "student").
  - `hardware_specs`: String (optional, e.g., "RTX 3060").
- **Server:** Create `index.ts` that runs the Hono server on port 4000.
- **CORS:** Allow requests from `http://localhost:3000` (Docusaurus).

### 2. `src/components/Auth/` (Frontend Components)
- **`AuthContext.tsx`**: A React Context provider wrapping the app.
- **`SignupForm.tsx`**: A form with:
  - Name, Email, Password.
  - **Dropdown:** "What is your background?" (Software/Hardware).
  - **Input:** "Do you have an NVIDIA GPU?"
- **`UserMenu.tsx`**: A component for the Navbar showing "Hello, [Name]" or "Login".

### 3. `src/components/PersonalizedContent.tsx` (Requirement #6)
- **Props:** `software` (Markdown), `hardware` (Markdown).
- **Logic:**
  - Get `user` from `useSession`.
  - If `user.background === 'software'`, render the `software` content.
  - If `user.background === 'hardware'`, render the `hardware` content.
  - Default: Show both side-by-side or a "General" view.

### 4. `src/components/TranslationButton.tsx` (Requirement #7)
- **UI:** A button "Translate to Urdu" at the top of pages.
- **Logic:**
  - On Click: Send the current page's markdown text to the FastAPI backend (`http://localhost:8000/translate`).
  - Display the returned Urdu text in a modal or replace the content.