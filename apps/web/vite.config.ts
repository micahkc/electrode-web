import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';
import { viteStaticCopy } from 'vite-plugin-static-copy';

const cesiumSource = new URL('../../node_modules/cesium/Build/Cesium', import.meta.url).pathname;
const cesiumBaseUrl = 'cesium';

export default defineConfig({
  define: {
    CESIUM_BASE_URL: JSON.stringify(`/${cesiumBaseUrl}`)
  },
  plugins: [
    sveltekit(),
    viteStaticCopy({
      targets: [
        { src: `${cesiumSource}/Workers`, dest: cesiumBaseUrl },
        { src: `${cesiumSource}/ThirdParty`, dest: cesiumBaseUrl },
        { src: `${cesiumSource}/Assets`, dest: cesiumBaseUrl },
        { src: `${cesiumSource}/Widgets`, dest: cesiumBaseUrl }
      ]
    })
  ],
  worker: {
    format: 'es'
  }
});
