import adapter from '@sveltejs/adapter-static';

function deploymentBasePath() {
  const base = process.env.ELECTRODE_WEB_BASE_PATH ?? '';
  if (base === '') return '';
  if (!base.startsWith('/') || base.endsWith('/')) {
    throw new Error('ELECTRODE_WEB_BASE_PATH must be empty or a root-relative path without a trailing slash');
  }
  return base;
}

/** @type {import('@sveltejs/kit').Config} */
const config = {
  kit: {
    adapter: adapter({
      fallback: 'index.html'
    }),
    // Keep local Ground Station builds rooted at "/" by default. GitHub Pages
    // builds set ELECTRODE_WEB_BASE_PATH=/electrode-web in CI.
    paths: {
      base: deploymentBasePath(),
      relative: true
    }
  }
};

export default config;
