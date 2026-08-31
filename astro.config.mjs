// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

const SITE_BASE = '/WildBridge/';

/**
 * Prefix root-relative links in Markdown content with `SITE_BASE` so internal
 * links keep working when the site is served from a subpath
 * (GitHub Pages project sites).
 */
function addBaseToLinks() {
	return (tree) => {
		walk(tree);
		function walk(node) {
			if (!node || typeof node !== 'object') return;
			if (
				node.type === 'link' &&
				typeof node.url === 'string' &&
				node.url.startsWith('/') &&
				!node.url.startsWith('//') &&
				!node.url.startsWith(SITE_BASE)
			) {
				node.url = SITE_BASE + node.url.slice(1);
			}
			if (Array.isArray(node.children)) {
				for (const child of node.children) walk(child);
			}
		}
	};
}

// https://astro.build/config
export default defineConfig({
	site: 'https://wilddrone.github.io',
	base: SITE_BASE,
	markdown: {
		remarkPlugins: [addBaseToLinks],
	},
	integrations: [
		starlight({
			title: 'WildBridge',
			logo: {
				src: './docs/images/WildBridge_icon.png',
			},
			social: [
				{ icon: 'github', label: 'GitHub', href: 'https://github.com/WildDrone/WildBridge' },
				{ icon: 'youtube', label: 'Demo videos', href: 'https://www.youtube.com/watch?v=PzHnbgxLaSU' },
			],
			editLink: {
				baseUrl: 'https://github.com/WildDrone/WildBridge/edit/main/',
			},
			lastUpdated: true,
			customCss: ['./src/styles/custom.css'],
			components: {
				PageTitle: './src/components/PageTitle.astro',
			},
			head: [
				{
					tag: 'link',
					attrs: { rel: 'preconnect', href: 'https://fonts.googleapis.com' },
				},
				{
					tag: 'link',
					attrs: { rel: 'preconnect', href: 'https://fonts.gstatic.com', crossorigin: true },
				},
			],
			sidebar: [
				{
					label: 'Start here',
					items: [
						{ label: 'Getting Started', slug: 'getting-started' },
						{ label: 'Ground Station', slug: 'groundstation' },
					],
				},
				{
					label: 'Interfaces',
					items: [
						{ label: 'HTTP API', slug: 'http-api' },
						{ label: 'Telemetry', slug: 'telemetry' },
						{ label: 'MAVLink 2', slug: 'mavlink' },
						{ label: 'ROS 2', slug: 'ros' },
					],
				},
				{
					label: 'Operations',
					items: [
						{ label: 'Logs & Troubleshooting', slug: 'operations' },
						{ label: 'Field Test', slug: 'field-test' },
					],
				},
			],
		}),
	],
});
