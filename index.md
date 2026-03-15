---
layout: default
title: Home
---

<h1>Recent Posts</h1>

<div class="post-list">
  {% for post in site.posts limit:5 %}
  <a href="{{ post.url }}" style="
    display: block;
    text-decoration: none;
    color: inherit;
  ">
    <div style="
      border: 1px solid #ddd;
      border-radius: 8px;
      padding: 1rem;
      margin-bottom: 1rem;
      transition: background-color 0.15s ease, box-shadow 0.15s ease;
    "
    onmouseover="this.style.boxShadow='0 4px 12px rgba(0,0,0,0.08)'"
    onmouseout="this.style.boxShadow='none'"
    >
      <h2 style="margin-top: 0;">
        {{ post.title }}
      </h2>

      <small>{{ post.date | date: "%B %d, %Y" }}</small>

      {% if post.custom_excerpt %}
        <p>{{ post.custom_excerpt }}</p>
      {% elsif post.excerpt %}
        <p>{{ post.excerpt | strip_html }}</p>
      {% endif %}
    </div>
  </a>
  {% endfor %}
</div>
